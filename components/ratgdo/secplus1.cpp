
#ifdef PROTOCOL_SECPLUSV1

#include "secplus1.h"
#include "ratgdo.h"

#include "esphome/core/gpio.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/scheduler.h"

namespace esphome::ratgdo {
namespace secplus1 {

    using namespace scheduler_ids;
    static const char* const TAG = "ratgdo_secplus1";
    // Multi-toggle dance legs now trigger on confirmed commit-CHANGES (see
    // the commit branch), so this expiry must cover press-to-committed-state
    // latency: press-to-motion ~0.5-1s plus the two-poll confirm — up to
    // ~3.4s at the 1Hz emulation status cadence. Legs check the exact state
    // they await AND every explicit door command clears pending legs
    // (command supersedes chain), so the wide window cannot fire a leg on a
    // non-matching state or over a newer command — it only reaps chains when
    // nothing happens at all.
    static constexpr uint32_t DOOR_STATE_CALLBACK_TIMEOUT = 7000;
    // After a light command, ignore polled light status this long so the opener's
    // reporting latency cannot bounce the reported light state on/off/on while it
    // settles (Security+1 light read-back fix).
    static constexpr uint32_t LIGHT_CMD_SUPPRESS_MS = 2000;
    // How long the bus must stay silent after boot before we conclude no wall
    // panel is actively transmitting and allow our first transmit. A present,
    // running panel is heard within milliseconds, so this only delays the
    // first command on a silent bus.
    static constexpr uint32_t BUS_QUIET_GRACE_MS = 2000;
    // A STOP can arrive before the two-poll status debounce has confirmed the
    // door is actually moving (short move-to-position). Retry on this cadence
    // until motion confirms or a terminal state shows the door never moved.
    static constexpr uint32_t STOP_RETRY_INTERVAL_MS = 250;
    static constexpr uint8_t STOP_RETRY_MAX_TRIES = 12; // ~3s: covers 2-poll confirm even at emulation's 1Hz door-status cadence
    static constexpr uint8_t STOP_RETRY_MAX_TRIES_REVERSAL = 20; // ~5s: reversal-to-OPENING-confirm measured ~2.5-3s; margin for heavy/cold doors
    // A direct stop may only trust a moving state confirmed at most this long
    // ago. A genuinely moving door re-confirms every poll (~250-1000ms); a
    // stale OPENING/CLOSING can mean the door has already latched a limit,
    // where a toggle would START travel instead of stopping it (seen on
    // hardware: a stop racing the open limit became a full close).
    static constexpr uint32_t STOP_STATE_FRESH_MS = 1200;
    // Acknowledged stops: after a stop toggle is transmitted, verify the door
    // actually ceased moving. On a shared bus with a wall panel, a toggle can
    // be lost outright (seen on hardware: a mid-travel stop was transmitted
    // and the opener kept going to the limit). Suspicion is raised only after
    // the delay below (past worst-case tx pacing + press-to-stop lag + the
    // terminal-state confirm), and a re-send additionally requires a
    // same-direction status response SOLICITED AND RECEIVED at verify time —
    // never cached knowledge. CLOSING gets a longer delay: a successful
    // reversal keeps answering CLOSING while decelerating (~2.5-3s to the
    // OPENING confirm on the measured hardware).
    static constexpr uint32_t STOP_VERIFY_DELAY_MS = 3000;
    // delay + recheck (5200ms) strictly exceeds the reversal retry budget
    // (20 x 250ms = 5000ms), so the verify can never re-toggle a reversal the
    // retry still considers in progress.
    static constexpr uint32_t STOP_VERIFY_DELAY_CLOSING_MS = 4500;
    static constexpr uint32_t STOP_VERIFY_RECHECK_MS = 700; // solicited response round-trip
    static constexpr uint8_t STOP_VERIFY_MAX_ATTEMPTS = 2; // re-toggles after the initial one
    // A terminal (OPEN/CLOSED) report while the door is confirmed MOVING must
    // be sustained — minimum consecutive readings AND minimum wall-clock span
    // — before it commits. Seen on hardware three times in one evening, each
    // killing an armed position stop mid-travel: the opener emits a phantom
    // limit report during real travel. Measured phantoms arrive as BURSTS of
    // 1-2 identical readings spanning 56-600ms (so read-counting alone loses:
    // a 2-read burst beat the count-2 rule), while a real latch reports
    // itself indefinitely. Time is the separating axis: no observed burst
    // approaches the span below; a real latch merely confirms ~1.5s later,
    // and during the pending window the moving state goes stale, so stop
    // commands take the safe deferred path.
    static constexpr uint8_t TERMINAL_FROM_MOVING_MIN_READS = 2;
    static constexpr uint32_t TERMINAL_FROM_MOVING_MIN_SPAN_MS = 1500;

    void Secplus1::setup(RATGDOComponent* ratgdo, Scheduler* scheduler, InternalGPIOPin* rx_pin, InternalGPIOPin* tx_pin)
    {
        this->ratgdo_ = ratgdo;
        this->scheduler_ = scheduler;
        this->tx_pin_ = tx_pin;
        this->rx_pin_ = rx_pin;

        this->uart_.begin(1200, RATGDO_UART_8E1, rx_pin->get_pin(), tx_pin->get_pin(), true);

        // Anchor the quiet-bus grace window at listen start so it measures
        // actual listening time; sync() re-anchors it shortly after.
        this->wall_panel_emulation_start_ = millis();

        this->traits_.set_features(HAS_DOOR_STATUS | HAS_LIGHT_TOGGLE | HAS_LOCK_TOGGLE);
    }

    void Secplus1::loop()
    {
        auto rx_cmd = this->read_command();
        if (rx_cmd) {
            this->handle_command(rx_cmd.value());
        }
        auto tx_cmd = this->pending_tx();
        if (
            // After boot, wait until we've heard the wall panel before transmitting,
            // so we slot into the bus instead of colliding with an in-progress
            // message. If the bus stays silent past the grace period there is no
            // active panel (none installed, or it is still booting) and it is safe
            // to transmit — without this, a no-panel setup would hold its first
            // command until emulation engages.
            (this->last_rx_ != 0 || millis() - this->wall_panel_emulation_start_ > BUS_QUIET_GRACE_MS) && (millis() - this->last_tx_) > 200 && // don't send twice in a period
            (millis() - this->last_rx_) > 50 && // time to send it
            tx_cmd && // have pending command
            !(this->flags_.is_0x37_panel && tx_cmd.value() == CommandType::TOGGLE_LOCK_PRESS) && this->wall_panel_emulation_state_ != WallPanelEmulationState::RUNNING) {
            this->do_transmit_if_pending();
        }
    }

    void Secplus1::dump_config()
    {
        ESP_LOGCONFIG(TAG, "  Protocol: SEC+ v1");
    }

    void Secplus1::on_shutdown()
    {
        this->uart_.on_shutdown();
    }

    void Secplus1::sync()
    {
        this->wall_panel_emulation_state_ = WallPanelEmulationState::WAITING;
        this->wall_panel_emulation_start_ = millis();
        this->flags_.wall_panel_starting = false;
        this->door_state = DoorState::UNKNOWN;
        this->light_state = LightState::UNKNOWN;
        this->scheduler_->cancel_timeout(this->ratgdo_, TIMEOUT_WALL_PANEL_EMULATION);
        this->wall_panel_emulation();

        this->ratgdo_->set_timeout(70000, [this] {
            if (this->door_state == DoorState::UNKNOWN) {
                ESP_LOGW(TAG, "Triggering sync failed actions.");
                this->ratgdo_->sync_failed = true;
            }
        });
    }

    void Secplus1::wall_panel_emulation(size_t index)
    {
        if (this->flags_.wall_panel_starting) {
            this->wall_panel_emulation_state_ = WallPanelEmulationState::WAITING;
        } else if (this->wall_panel_emulation_state_ == WallPanelEmulationState::WAITING) {
            ESP_LOGD(TAG, "Looking for security+ 1.0 wall panel...");

            if (this->door_state != DoorState::UNKNOWN || this->light_state != LightState::UNKNOWN) {
                ESP_LOG1(TAG, "Wall panel detected");
                return;
            }
            if (millis() - this->wall_panel_emulation_start_ > 60000 && !this->flags_.wall_panel_starting) {
#ifdef RATGDO_NO_EMULATION
                // This install has a wall panel permanently wired: never
                // become a second transmitter on the bus, no matter how
                // silent it is. A panel that boots slower than the window
                // (889LM: 2-3 min) or reboots later just resumes feeding us
                // status when it speaks; nothing here needs to run again.
                // (Without this, RUNNING is permanent — no code path ever
                // exits emulation short of a reboot.)
                ESP_LOGW(TAG, "No wall panel heard after 60s — emulation DISABLED by build flag; waiting passively for the panel.");
                return;
#else
                ESP_LOGD(TAG, "No wall panel detected. Switching to emulation mode.");
                this->wall_panel_emulation_state_ = WallPanelEmulationState::RUNNING;
#endif
            }
            this->scheduler_->set_timeout(this->ratgdo_, TIMEOUT_WALL_PANEL_EMULATION, 2000, [this] {
                this->wall_panel_emulation();
            });
            return;
        } else if (this->wall_panel_emulation_state_ == WallPanelEmulationState::RUNNING) {
#ifdef USE_ESP8266
            // ESP_LOG2(TAG, "[Wall panel emulation] Sending byte: [%02X]", progmem_read_byte(&secplus1_states[index]));
#else
            // ESP_LOG2(TAG, "[Wall panel emulation] Sending byte: [%02X]", secplus1_states[index]);
#endif

            if (index < 15 || !this->do_transmit_if_pending()) {
#ifdef USE_ESP8266
                this->transmit_byte(progmem_read_byte(&secplus1_states[index]));
#else
                this->transmit_byte(secplus1_states[index]);
#endif
                // gdo response simulation for testing
#ifdef USE_ESP8266
                // auto resp = progmem_read_byte(&secplus1_states[index]) == 0x39 ? 0x00 :
                //             progmem_read_byte(&secplus1_states[index]) == 0x3A ? 0x5C :
                //             progmem_read_byte(&secplus1_states[index]) == 0x38 ? 0x52 : 0xFF;
#else
                // auto resp = secplus1_states[index] == 0x39 ? 0x00 :
                //             secplus1_states[index] == 0x3A ? 0x5C :
                //             secplus1_states[index] == 0x38 ? 0x52 : 0xFF;
#endif
                // if (resp != 0xFF) {
                //     this->transmit_byte(resp, true);
                // }

                index += 1;
                if (index == 19) { // cycle all 4 poll items (15..18 = 0x38,0x3A,0x39,0x3A)
                    index = 15;
                }
            }
            this->scheduler_->set_timeout(this->ratgdo_, TIMEOUT_WALL_PANEL_EMULATION, 250, [this, index] {
                this->wall_panel_emulation(index);
            });
        }
    }

    void Secplus1::light_action(LightAction action)
    {
        ESP_LOG1(TAG, "Light action: %s", LOG_STR_ARG(LightAction_to_string(action)));
        if (action == LightAction::UNKNOWN) {
            return;
        }
        if (
            action == LightAction::TOGGLE || (action == LightAction::ON && this->light_state == LightState::OFF) || (action == LightAction::OFF && this->light_state == LightState::ON)) {
            // Mark the command time so QUERY_OTHER_STATUS briefly ignores stale
            // readback and does not bounce the reported light state while the
            // opener settles after the toggle.
            this->light_command_at_ = millis();
            this->toggle_light();
        }
    }

    void Secplus1::lock_action(LockAction action)
    {
        ESP_LOG1(TAG, "Lock action: %s", LOG_STR_ARG(LockAction_to_string(action)));
        if (action == LockAction::UNKNOWN) {
            return;
        }
        if (
            action == LockAction::TOGGLE || (action == LockAction::LOCK && this->lock_state == LockState::UNLOCKED) || (action == LockAction::UNLOCK && this->lock_state == LockState::LOCKED)) {
            this->toggle_lock();
        }
    }

    void Secplus1::set_door_state_expiry()
    {
        this->scheduler_->set_timeout(this->ratgdo_, TIMEOUT_SECPLUS1_DOOR_STATE_EXPIRY, DOOR_STATE_CALLBACK_TIMEOUT, [this]() {
            ESP_LOGW(TAG, "Door state callback expired, clearing");
            this->on_door_state_.clear();
        });
    }

    void Secplus1::cancel_door_state_expiry()
    {
        this->scheduler_->cancel_timeout(this->ratgdo_, TIMEOUT_SECPLUS1_DOOR_STATE_EXPIRY);
    }

    void Secplus1::door_action(DoorAction action)
    {
        ESP_LOG1(TAG, "Door action: %s, door state: %s", LOG_STR_ARG(DoorAction_to_string(action)), LOG_STR_ARG(DoorState_to_string(this->door_state)));
        if (action == DoorAction::UNKNOWN) {
            return;
        }

        // An explicit door command supersedes any in-flight multi-toggle
        // dance: a surviving leg would fire on a later commit and toggle over
        // the new command — a user STOP could be silently discarded when the
        // leg's toggle_door cancels the freshly-armed stop retry.
        this->on_door_state_.clear();
        this->cancel_door_state_expiry();

        const uint32_t double_toggle_delay = 1000;
        if (action == DoorAction::TOGGLE) {
            this->toggle_door();
        } else if (action == DoorAction::OPEN) {
            if (this->door_state == DoorState::CLOSED || this->door_state == DoorState::CLOSING) {
                this->toggle_door();
            } else if (this->door_state == DoorState::STOPPED) {
                this->toggle_door(); // this starts closing door
                // When the outer callback fires and registers an inner callback,
                // on_door_state() replaces the outer's expiry with a new one for
                // the inner callback (same timeout ID = replace, not add).
                this->on_door_state([this](DoorState s) {
                    if (s == DoorState::CLOSING) {
                        // this changes direction of the door on some openers, on others it stops it
                        this->toggle_door();
                        this->on_door_state([this](DoorState s) {
                            if (s == DoorState::STOPPED) {
                                this->toggle_door();
                            }
                        });
                    }
                });
            }
        } else if (action == DoorAction::CLOSE) {
            if (this->door_state == DoorState::OPEN) {
                this->toggle_door();
            } else if (this->door_state == DoorState::OPENING) {
                this->toggle_door(); // this switches to stopped
                // another toggle needed to close
                this->on_door_state([this](DoorState s) {
                    if (s == DoorState::STOPPED) {
                        this->toggle_door();
                    }
                });
            } else if (this->door_state == DoorState::STOPPED) {
                this->toggle_door();
            }
        } else if (action == DoorAction::STOP) {
            // Direct stops require FRESH knowledge that the door is moving;
            // stale or unconfirmed state routes through the deferred retry,
            // which acts only on a fresh post-request confirmation and lets a
            // latched door expire untouched.
            // Fresh = recently confirmed AND no contrary reading in flight.
            // The second term closes the just-latched race: at a real limit
            // the last moving commit can still be <1200ms old while terminal
            // readings are pending the sustained-evidence gate — a direct
            // toggle there would command a latched door. Contrary maybe =>
            // the stop takes the deferred path, which gives up cleanly at a
            // terminal commit.
            bool state_fresh = millis() - this->door_state_confirmed_at_ <= STOP_STATE_FRESH_MS
                && this->maybe_door_state == this->door_state;
            if (this->door_state == DoorState::OPENING && state_fresh) {
                this->toggle_door();
                this->schedule_stop_verify(DoorState::OPENING, STOP_VERIFY_MAX_ATTEMPTS);
            } else if (this->door_state == DoorState::CLOSING && state_fresh) {
                this->toggle_door(); // this switches to opening

                // Another toggle is needed once the door is confirmed OPENING.
                // The old one-shot on_door_state callback lost it — the physical
                // reversal can take >2s to confirm (beyond the callback's 2s
                // expiry) and any intermediate state consumes the one-shot — so
                // the door ran fully open instead of stopping (seen on
                // hardware). Defer it via the stop retry instead: acts only on
                // a FRESH confirmed OPENING, survives intermediates, and openers
                // whose toggle stops (rather than reverses) a closing door just
                // confirm a terminal state and let the retry expire harmlessly.
                this->stop_requested_at_ = millis();
                this->schedule_stop_retry(STOP_RETRY_MAX_TRIES_REVERSAL, DoorState::OPENING);
                // Also verify the reverse toggle itself took: if the opener is
                // still verifiably CLOSING (solicited confirmation, not cached
                // state) well past the reversal's deceleration window, the
                // toggle was lost — re-send it and re-arm the reversal retry
                // (the verify's re-send necessarily cancels the pending one).
                this->schedule_stop_verify(DoorState::CLOSING, STOP_VERIFY_MAX_ATTEMPTS);
            } else {
                // Motion may have been commanded, but the two-poll status
                // debounce hasn't confirmed OPENING/CLOSING yet — a short
                // move-to-position fires its STOP inside that window. Without
                // this branch the stop silently no-ops and the door runs to its
                // travel limit (seen on hardware). Arm unconditionally (NOT
                // gated on door_moving: a stale terminal re-confirm poll can
                // clear that flag before real motion confirms) — if the door
                // truly is not moving the retry ticks transmit only status
                // queries and expire harmlessly.
                this->stop_requested_at_ = millis();
                this->schedule_stop_retry(STOP_RETRY_MAX_TRIES);
            }
        }
    }

    void Secplus1::schedule_stop_retry(uint8_t tries, DoorState await_state)
    {
        if (tries == 0) {
            ESP_LOGW(TAG, "Deferred stop expired without confirming door motion");
            return;
        }
        this->scheduler_->set_timeout(this->ratgdo_, TIMEOUT_SECPLUS1_STOP_RETRY, STOP_RETRY_INTERVAL_MS, [this, tries, await_state] {
            // Act only on a motion confirmation FRESHER than the stop request:
            // a stale pre-stop confirmation could otherwise toggle a door that
            // has just come to rest at its travel limit, starting a new move.
            // (now - confirmed_at <= now - requested_at  <=>  confirmed after
            // requested; unsigned math keeps this millis()-rollover safe.)
            // await_state == UNKNOWN accepts either moving state (plain stop);
            // the reverse-stop dance passes OPENING so a fresh CLOSING
            // re-confirm cannot trigger a second reversal toggle.
            uint32_t now = millis();
            bool awaited = await_state == DoorState::UNKNOWN
                ? (this->door_state == DoorState::OPENING || this->door_state == DoorState::CLOSING)
                : this->door_state == await_state;
            if (awaited && (now - this->door_state_confirmed_at_) <= (now - this->stop_requested_at_)) {
                // Motion (re)confirmed after the stop request; run the real
                // stop sequence via the OPENING/CLOSING branches above.
                this->door_action(DoorAction::STOP);
            } else {
                // Solicit a fresh door-status read so confirmation cannot
                // starve (0x37 injection and emulation polling may otherwise
                // idle once door_moving is cleared), then wait another tick.
                this->enqueue_transmit(CommandType::QUERY_DOOR_STATUS);
                this->last_status_query_ = millis();
                this->schedule_stop_retry(tries - 1, await_state);
            }
        });
    }

    // Acknowledged stop: verify a transmitted stop toggle actually took.
    // Two phases, so a re-send is only ever based on evidence gathered AFTER
    // suspicion is raised — never on cached state that a decelerating or
    // just-latched door can leave behind:
    //   1. At the (direction-specific) verify delay, if BOTH the debounced and
    //      the in-flight (maybe) state still show `direction` with a fresh
    //      confirm, solicit a door-status read.
    //   2. A short recheck later, re-send the toggle only if a same-direction
    //      confirmation arrived AFTER the solicitation. A door that stopped,
    //      reversed, or latched answers the solicited query with its true
    //      state, which vetoes the re-send (no post-latch toggles). A missed
    //      response also vetoes — detection is best-effort, safety absolute;
    //      the un-stopped door then runs to a limit exactly as before this
    //      layer existed.
    void Secplus1::schedule_stop_verify(DoorState direction, uint8_t attempts)
    {
        uint32_t delay = direction == DoorState::CLOSING ? STOP_VERIFY_DELAY_CLOSING_MS : STOP_VERIFY_DELAY_MS;
        this->scheduler_->set_timeout(this->ratgdo_, TIMEOUT_SECPLUS1_STOP_VERIFY, delay, [this, direction, attempts] {
            bool suspect = this->door_state == direction
                && this->maybe_door_state == direction
                && millis() - this->door_state_confirmed_at_ <= STOP_STATE_FRESH_MS;
            if (!suspect) {
                return; // stop took effect (or a contrary reading is already in flight)
            }
            uint32_t solicited_at = millis();
            this->enqueue_transmit(CommandType::QUERY_DOOR_STATUS);
            this->last_status_query_ = solicited_at;
            this->scheduler_->set_timeout(this->ratgdo_, TIMEOUT_SECPLUS1_STOP_VERIFY, STOP_VERIFY_RECHECK_MS, [this, direction, attempts, solicited_at] {
                uint32_t now = millis();
                bool confirmed_after_solicit = this->door_state == direction
                    && this->maybe_door_state == direction
                    && (now - this->door_state_confirmed_at_) < (now - solicited_at); // STRICT: same-ms pre-solicit reads don't count
                if (!confirmed_after_solicit) {
                    return; // true state says not moving in `direction` (or no response) — leave it alone
                }
                if (attempts == 0) {
                    ESP_LOGW(TAG, "Stop toggle repeatedly ineffective; door will run to its limit");
                    return;
                }
                ESP_LOGW(TAG, "Stop toggle did not take (door still %s); re-sending",
                    LOG_STR_ARG(DoorState_to_string(direction)));
                this->toggle_door(); // cancels the (already-fired) verify and the reversal retry
                if (direction == DoorState::CLOSING) {
                    // toggle_door's cancel killed the awaiting-OPENING retry
                    // that owns the reversal's second toggle — re-arm it for
                    // the re-sent reverse toggle.
                    this->stop_requested_at_ = millis();
                    this->schedule_stop_retry(STOP_RETRY_MAX_TRIES_REVERSAL, DoorState::OPENING);
                }
                this->schedule_stop_verify(direction, attempts - 1);
            });
        });
    }

    void Secplus1::toggle_light()
    {
        this->enqueue_transmit(CommandType::TOGGLE_LIGHT_PRESS);
    }

    void Secplus1::toggle_lock()
    {
        this->enqueue_transmit(CommandType::TOGGLE_LOCK_PRESS);
    }

    void Secplus1::toggle_door()
    {
        // Any door toggle — a new user command or a step of an in-flight
        // command chain — supersedes a deferred stop still waiting for motion
        // to confirm: the stop must not fire into that motion. Likewise a
        // pending stop-verify belongs to a superseded toggle. (The verify
        // lambda re-arms itself AFTER calling toggle_door, so its own chain
        // survives this cancel.)
        this->scheduler_->cancel_timeout(this->ratgdo_, TIMEOUT_SECPLUS1_STOP_RETRY);
        this->scheduler_->cancel_timeout(this->ratgdo_, TIMEOUT_SECPLUS1_STOP_VERIFY);
        this->enqueue_transmit(CommandType::TOGGLE_DOOR_PRESS);
        this->enqueue_transmit(CommandType::QUERY_DOOR_STATUS);
        if (this->door_state == DoorState::STOPPED || this->door_state == DoorState::OPEN || this->door_state == DoorState::CLOSED) {
            this->flags_.door_moving = true;
        }
    }

    Result Secplus1::call(Args args)
    {
        return { };
    }

    optional<RxCommand> Secplus1::read_command()
    {
        if (!this->flags_.rx_reading_msg) {
            while (this->uart_.available()) {
                uint8_t ser_byte = this->uart_.read();
                this->last_rx_ = millis();

                if (ser_byte < 0x30 || ser_byte > 0x3A) {
                    char hex[format_hex_pretty_size(1)];
                    ESP_LOG2(TAG, "[%d] Ignoring byte [%s], baud: %d", millis(), format_hex_pretty_to(hex, &ser_byte, 1), this->uart_.baudRate());
                    this->rx_byte_count_ = 0;
                    continue;
                }
                this->rx_packet_[this->rx_byte_count_++] = ser_byte;
                {
                    char hex[format_hex_pretty_size(1)];
                    ESP_LOG2(TAG, "[%d] Received byte: [%s]", millis(), format_hex_pretty_to(hex, &ser_byte, 1));
                }
                this->flags_.rx_reading_msg = true;

                if (ser_byte == 0x37 || (ser_byte >= 0x30 && ser_byte <= 0x35)) {
                    this->rx_packet_[this->rx_byte_count_++] = 0;
                    this->flags_.rx_reading_msg = false;
                    this->rx_byte_count_ = 0;
                    {
                        char hex[format_hex_pretty_size(1)];
                        ESP_LOG2(TAG, "[%d] Received command: [%s]", millis(), format_hex_pretty_to(hex, &this->rx_packet_[0], 1));
                    }
                    return this->decode_packet(this->rx_packet_);
                }

                break;
            }
        }
        if (this->flags_.rx_reading_msg) {
            while (this->uart_.available()) {
                uint8_t ser_byte = this->uart_.read();
                this->last_rx_ = millis();
                this->rx_packet_[this->rx_byte_count_++] = ser_byte;
                {
                    char hex[format_hex_pretty_size(1)];
                    ESP_LOG2(TAG, "[%d] Received byte: [%s]", millis(), format_hex_pretty_to(hex, &ser_byte, 1));
                }

                if (this->rx_byte_count_ == RX_LENGTH) {
                    this->flags_.rx_reading_msg = false;
                    this->rx_byte_count_ = 0;
                    this->print_rx_packet(this->rx_packet_);
                    return this->decode_packet(this->rx_packet_);
                }
            }

            if (millis() - this->last_rx_ > 100) {
                // if we have a partial packet and it's been over 100ms since last byte was read,
                // the rest is not coming (a full packet should be received in ~20ms),
                // discard it so we can read the following packet correctly
                {
                    char hex[format_hex_pretty_size(1)];
                    ESP_LOGW(TAG, "[%d] Discard incomplete packet: [%s ...]", millis(), format_hex_pretty_to(hex, &this->rx_packet_[0], 1));
                }
                this->flags_.rx_reading_msg = false;
                this->rx_byte_count_ = 0;
            }
        }

        return { };
    }

    void Secplus1::print_rx_packet(const RxPacket& packet) const
    {
        constexpr size_t hex_size = format_hex_pretty_size(RX_LENGTH);
        char hex[hex_size];
        ESP_LOG2(TAG, "[%d] Received packet: [%s]", millis(), format_hex_pretty_to(hex, packet, RX_LENGTH));
    }

    void Secplus1::print_tx_packet(const TxPacket& packet) const
    {
        constexpr size_t hex_size = format_hex_pretty_size(TX_LENGTH);
        char hex[hex_size];
        ESP_LOG2(TAG, "[%d] Sending packet: [%s]", millis(), format_hex_pretty_to(hex, packet, TX_LENGTH));
    }

    optional<RxCommand> Secplus1::decode_packet(const RxPacket& packet) const
    {
        CommandType cmd_type = to_CommandType(packet[0], CommandType::UNKNOWN);
        return RxCommand { cmd_type, packet[1] };
    }

    // unknown meaning of observed command-responses:
    // 40 00 and 40 80
    // 53 01
    // C0 3F
    // F8 3F
    // FE 3F

    void Secplus1::handle_command(const RxCommand& cmd)
    {
        if (cmd.req == CommandType::TOGGLE_DOOR_RELEASE || cmd.resp == 0x31) {
            if (this->wall_panel_emulation_state_ == WallPanelEmulationState::WAITING) {
                ESP_LOGD(TAG, "wall panel is starting");
                this->flags_.wall_panel_starting = true;
            }
        } else if (cmd.req == CommandType::QUERY_DOOR_STATUS) {
            DoorState door_state;
            auto val = cmd.resp & 0x7;
            // 000 0x0 stopped
            // 001 0x1 opening
            // 010 0x2 open
            // 100 0x4 closing
            // 101 0x5 closed
            // 110 0x6 stopped

            if (val == 0x2) {
                door_state = DoorState::OPEN;
            } else if (val == 0x5) {
                door_state = DoorState::CLOSED;
            } else if (val == 0x0 || val == 0x6) {
                door_state = DoorState::STOPPED;
            } else if (val == 0x1) {
                door_state = DoorState::OPENING;
            } else if (val == 0x4) {
                door_state = DoorState::CLOSING;
            } else {
                door_state = DoorState::UNKNOWN;
            }

            // Streak of consecutive identical readings, this one included,
            // and the wall-clock start of the current run.
            if (door_state == this->maybe_door_state) {
                if (this->door_reading_streak_ < 100) {
                    this->door_reading_streak_++;
                }
            } else {
                // Streak broken. Breaking from a pending terminal straight
                // back to the committed moving state is the phantom-limit
                // signature — record the evidence.
                if ((this->maybe_door_state == DoorState::OPEN || this->maybe_door_state == DoorState::CLOSED)
                    && door_state == this->door_state
                    && (this->door_state == DoorState::OPENING || this->door_state == DoorState::CLOSING)) {
                    ESP_LOGW(TAG, "Discarded phantom %s report(s) while %s",
                        LOG_STR_ARG(DoorState_to_string(this->maybe_door_state)),
                        LOG_STR_ARG(DoorState_to_string(this->door_state)));
                }
                this->door_reading_streak_ = 1;
                this->door_reading_run_started_ = millis();
            }

            this->maybe_door_state = door_state;

            bool commit;
            if ((this->flags_.door_moving || this->door_state == DoorState::OPENING || this->door_state == DoorState::CLOSING)
                && (door_state == DoorState::OPEN || door_state == DoorState::CLOSED)) {
                // door_moving is in the condition so a garbled-read UNKNOWN
                // commit can't launder the moving state away and let a
                // phantom terminal commit by the normal rule (UNKNOWN doesn't
                // clear door_moving; only real terminal/stopped commits do).
                // Terminal-from-moving: sustained-evidence rule (see the
                // constants above). Applies on every panel type — the 0x37
                // instant-commit path is exactly where the phantoms landed.
                commit = this->door_reading_streak_ >= TERMINAL_FROM_MOVING_MIN_READS
                    && millis() - this->door_reading_run_started_ >= TERMINAL_FROM_MOVING_MIN_SPAN_MS;
            } else {
                // Normal rule, unchanged: 0x37 panels commit every reading,
                // others need the classic two-poll agreement.
                commit = this->flags_.is_0x37_panel || this->door_reading_streak_ >= 2;
            }
            if (commit) {
                DoorState prev_committed = this->door_state;
                this->door_state = door_state;
                this->door_state_confirmed_at_ = millis(); // freshness marker for the deferred-stop retry
                if (this->door_state == DoorState::STOPPED || this->door_state == DoorState::OPEN || this->door_state == DoorState::CLOSED) {
                    this->flags_.door_moving = false;
                }
                // Dance legs fire on confirmed commit-CHANGES, never on raw
                // readings: the first raw CLOSING can arrive ~300ms after a
                // press — before the opener physically moves or will accept
                // another press — and a leg fired there gets its toggle
                // swallowed, collapsing the chain (seen on hardware: a plain
                // OPEN from stopped closed the door instead). A commit means
                // sustained evidence; phantoms never commit, so they can no
                // longer consume pending legs either.
                // UNKNOWN commits (garbled responses) don't count as a state
                // change for the legs — no leg awaits UNKNOWN, and consuming
                // a pending chain on garbage would kill it for nothing.
                if (door_state != prev_committed && door_state != DoorState::UNKNOWN) {
                    this->on_door_state_.trigger(door_state);
                }
                this->ratgdo_->received(door_state);
            } else {
                ESP_LOG1(TAG, "Door maybe %s (%u reads, %ums), waiting to confirm",
                    LOG_STR_ARG(DoorState_to_string(door_state)), this->door_reading_streak_,
                    (unsigned)(millis() - this->door_reading_run_started_));
            }
        } else if (cmd.req == CommandType::QUERY_DOOR_STATUS_0x37) {
            this->flags_.is_0x37_panel = true;
            auto cmd = this->pending_tx();
            if (cmd && cmd.value() == CommandType::TOGGLE_LOCK_PRESS) {
                this->do_transmit_if_pending();
            } else {
                // inject door status request
                if (flags_.door_moving || (millis() - this->last_status_query_ > 10000)) {
                    this->transmit_byte(static_cast<uint8_t>(CommandType::QUERY_DOOR_STATUS));
                    this->last_status_query_ = millis();
                }
            }
        } else if (cmd.req == CommandType::QUERY_OTHER_STATUS) {
            LightState light_state = to_LightState((cmd.resp >> 2) & 1, LightState::UNKNOWN);

            if (this->light_command_at_ != 0 && millis() - this->light_command_at_ < LIGHT_CMD_SUPPRESS_MS) {
                // A light command was just issued and the opener is still
                // settling. Track the reading but do NOT apply it, so the
                // reported state doesn't bounce on/off/on before the opener
                // reflects the toggle.
                this->maybe_light_state = light_state;
            } else if (!this->flags_.is_0x37_panel && light_state != this->maybe_light_state) {
                this->maybe_light_state = light_state;
            } else {
                this->light_state = light_state;
                this->ratgdo_->received(light_state);
            }

            LockState lock_state = to_LockState((~cmd.resp >> 3) & 1, LockState::UNKNOWN);
            if (!this->flags_.is_0x37_panel && lock_state != this->maybe_lock_state) {
                this->maybe_lock_state = lock_state;
            } else {
                this->lock_state = lock_state;
                this->ratgdo_->received(lock_state);
            }
        } else if (cmd.req == CommandType::OBSTRUCTION) {
            ObstructionState obstruction_state = cmd.resp == 0 ? ObstructionState::CLEAR : ObstructionState::OBSTRUCTED;
            this->ratgdo_->received(obstruction_state);
        } else if (cmd.req == CommandType::TOGGLE_LIGHT_PRESS) {
            // motion was detected, or the light toggle button was pressed
            // either way it's ok to trigger motion detection
            if (this->light_state == LightState::OFF) {
                this->ratgdo_->received(MotionState::DETECTED);
            }
        } else if (cmd.req == CommandType::TOGGLE_DOOR_PRESS) {
            this->ratgdo_->received(ButtonState::PRESSED);
        } else if (cmd.req == CommandType::TOGGLE_DOOR_RELEASE) {
            this->ratgdo_->received(ButtonState::RELEASED);
        }
    }

    bool Secplus1::do_transmit_if_pending()
    {
        auto cmd = this->pop_pending_tx();
        if (cmd) {
            if (cmd.value() == CommandType::TOGGLE_LIGHT_PRESS || cmd.value() == CommandType::TOGGLE_LIGHT_RELEASE) {
                // Key the readback-suppression window to the actual send time of
                // the last light byte — transmission can lag the request when the
                // bus is busy, and the window must not expire before the opener
                // has seen the full press/release sequence.
                this->light_command_at_ = millis();
            }
            this->enqueue_command_pair(cmd.value());
            this->transmit_byte(static_cast<uint32_t>(cmd.value()));
        }
        return cmd.has_value();
    }

    void Secplus1::enqueue_command_pair(CommandType cmd)
    {
        auto now = millis();
        if (cmd == CommandType::TOGGLE_DOOR_PRESS) {
            this->enqueue_transmit(CommandType::TOGGLE_DOOR_RELEASE, now + 500);
        } else if (cmd == CommandType::TOGGLE_LIGHT_PRESS) {
            // Some Security+1 openers act on every light button event, so a
            // single press+release nets ZERO change (the light flips, then
            // reverts ~500ms later on the lone release). The HomeKit ratgdo
            // firmware sends the press followed by TWO releases for exactly
            // this reason; mirror that so the net result is one clean toggle.
            this->enqueue_transmit(CommandType::TOGGLE_LIGHT_RELEASE, now + 250);
            this->enqueue_transmit(CommandType::TOGGLE_LIGHT_RELEASE, now + 500);
        } else if (cmd == CommandType::TOGGLE_LOCK_PRESS) {
            this->enqueue_transmit(CommandType::TOGGLE_LOCK_RELEASE, now + 3500);
        };
    }

    void Secplus1::enqueue_transmit(CommandType cmd, uint32_t time)
    {
        if (time == 0) {
            time = millis();
        }
        this->pending_tx_.push(TxCommand { cmd, time });
    }

    optional<CommandType> Secplus1::pending_tx()
    {
        if (this->pending_tx_.empty()) {
            return { };
        }
        auto cmd = this->pending_tx_.top();
        if (cmd.time > millis()) {
            return { };
        }
        return cmd.request;
    }

    optional<CommandType> Secplus1::pop_pending_tx()
    {
        auto cmd = this->pending_tx();
        if (cmd) {
            this->pending_tx_.pop();
        }
        return cmd;
    }

    void Secplus1::transmit_byte(uint32_t value)
    {
#ifdef RATGDO_RX_ONLY
        // Diagnostic partition build: the board listens and logs but is
        // incapable of putting a bit on the Sec+1 bus. If unattended door
        // activity continues in this mode, the ratgdo is exonerated and the
        // captured bus traffic shows what did it. No door/light/lock control
        // works while this is compiled in — that is the point.
        ESP_LOGW(TAG, "[RX-ONLY BUILD] suppressed TX of byte [%02X]", static_cast<uint8_t>(value));
        this->last_tx_ = millis();
        return;
#endif
        bool enable_rx = (value == 0x38) || (value == 0x39) || (value == 0x3A);
        if (!enable_rx) {
            this->uart_.enableIntTx(false);
        }
        this->uart_.write(value);
        this->last_tx_ = millis();
        if (!enable_rx) {
            this->uart_.enableIntTx(true);
        }
        {
            uint8_t byte_val = static_cast<uint8_t>(value);
            char hex[format_hex_pretty_size(1)];
            ESP_LOGD(TAG, "[%d] Sent byte: [%s]", millis(), format_hex_pretty_to(hex, &byte_val, 1));
        }
    }

} // namespace secplus1
} // namespace esphome::ratgdo

#endif // PROTOCOL_SECPLUSV1
