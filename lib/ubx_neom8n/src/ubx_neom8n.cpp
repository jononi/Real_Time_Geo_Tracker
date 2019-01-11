/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ubx.cpp
 *
 * U-Blox protocol implementation. Following u-blox 6/7/8 Receiver Description
 * including Prototol Specification.
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 *
 * @author Hannes Delago
 *   (rework, add ubx7+ compatibility)
 *
 * @author Ankur Dave
 *   (port to Particle Electron, specialize for ubx 8, simplify, add commands)
 *    https://github.com/ankurdave/IntervalTracker
 *
 * @see https://www2.u-blox.com/images/downloads/Product_Docs/u-blox6-GPS-GLONASS-QZSS-V14_ReceiverDescriptionProtocolSpec_Public_(GPS.G6-SW-12013).pdf
 * @see https://www.u-blox.com/sites/default/files/products/documents/u-bloxM8_ReceiverDescrProtSpec_%28UBX-13003221%29_Public.pdf
 */

#include <math.h>
#include <string.h>
#include "Particle.h"

#include "ubx_neom8n.h"

#define ASSISTNOW_TOKEN "1a-2b3c4d5e6f"
#define UBX_CONFIG_TIMEOUT  200     // ms, timeout for waiting on an ACK
#define UBX_PACKET_TIMEOUT	10		// ms, timeout for waiting on further data once some data has been received

#define MIN(X,Y)    ((X) < (Y) ? (X) : (Y))

/**** Trace macros, disable for production builds */
#define UBX_TRACE_PARSER(s, ...)    {/*Serial.printlnf(s, ## __VA_ARGS__);*/}  /* decoding progress in parse_char() */
#define UBX_TRACE_RXMSG(s, ...)     {Serial.printlnf(s, ## __VA_ARGS__);}  /* Rx msgs in payload_rx_done() */
#define UBX_TRACE_CONFIG(s, ...)    {Serial.printlnf(s, ## __VA_ARGS__);}
#define UBX_TRACE_ASSIST(s, ...)    {Serial.printlnf(s, ## __VA_ARGS__);}

/**** Warning macros, disable to save memory */
#define UBX_WARN(s, ...)        {Serial.printlnf(s, ## __VA_ARGS__);}

UBX_NEOM8N::UBX_NEOM8N(std::function<void(uint16_t, const ubx_buf_t &)> callback) :
    _ack_state(UBX_ACK_IDLE),
    _ack_waiting_msg(0),
    _aop_status(true),
    _last_assistnow_fetch_unixtime(0),
    _assistnow_ip(),
    _callback(callback) {

    decode_init();
}

UBX_NEOM8N::~UBX_NEOM8N() {}

bool UBX_NEOM8N::start(USARTSerial *uart, uint32_t baud, uint8_t en_pin) {
    // Start receiver
    serial = uart;
    serial->begin(baud);

    pinMode(en_pin, OUTPUT);
    digitalWrite(en_pin, HIGH); //using a modified Pololu S7V8F3 voltage regulator

    /* flush input and wait for at least 100 ms silence */
    decode_init();
    wait_for_silence(100);
    decode_init();

    /* Send a CFG-PRT message to set the UBX protocol for in and out */
    memset(&_buf.payload_tx_cfg_prt, 0, sizeof(_buf.payload_tx_cfg_prt));
    _buf.payload_tx_cfg_prt.portID       = UBX_TX_CFG_PRT_PORTID;
    _buf.payload_tx_cfg_prt.mode         = UBX_TX_CFG_PRT_MODE;
    _buf.payload_tx_cfg_prt.baudRate     = UBX_TX_CFG_PRT_BAUDRATE;
    _buf.payload_tx_cfg_prt.inProtoMask  = UBX_TX_CFG_PRT_INPROTOMASK;
    _buf.payload_tx_cfg_prt.outProtoMask = UBX_TX_CFG_PRT_OUTPROTOMASK;
    send_message(UBX_MSG_CFG_PRT, _buf.raw, sizeof(_buf.payload_tx_cfg_prt));

    wait_for_ack(UBX_MSG_CFG_PRT, UBX_CONFIG_TIMEOUT); // no ack expected

    /* Send a CFG-PM2 message ((to limit peak current: disabled) */
    memset(&_buf.payload_tx_cfg_pm2, 0, sizeof(_buf.payload_tx_cfg_pm2));
    _buf.payload_tx_cfg_pm2.version = 0x02;
    _buf.payload_tx_cfg_pm2.flags = 0x00021000; // not limiting current. To limit: 0x00021100
    _buf.payload_tx_cfg_pm2.updatePeriod = 1000; // update every 1000ms
    _buf.payload_tx_cfg_pm2.searchPeriod = 10000;// retry after 10s if acquisition fails
    send_message(UBX_MSG_CFG_PM2, _buf.raw, sizeof(_buf.payload_tx_cfg_pm2));

    if (!wait_for_ack(UBX_MSG_CFG_PM2, UBX_CONFIG_TIMEOUT)) UBX_WARN("No ack for CFG-PM2");


    /* Send a CFG-RATE message to define update rate */
    memset(&_buf.payload_tx_cfg_rate, 0, sizeof(_buf.payload_tx_cfg_rate));
    _buf.payload_tx_cfg_rate.measRate   = UBX_TX_CFG_RATE_MEASINTERVAL;
    _buf.payload_tx_cfg_rate.navRate    = UBX_TX_CFG_RATE_NAVRATE;
    _buf.payload_tx_cfg_rate.timeRef    = UBX_TX_CFG_RATE_TIMEREF;
    send_message(UBX_MSG_CFG_RATE, _buf.raw, sizeof(_buf.payload_tx_cfg_rate));

    if (!wait_for_ack(UBX_MSG_CFG_RATE, UBX_CONFIG_TIMEOUT)) UBX_WARN("No ack for CFG-RATE");

    /* send a NAV5 message to set the options for the internal filter */
    memset(&_buf.payload_tx_cfg_nav5, 0, sizeof(_buf.payload_tx_cfg_nav5));
    _buf.payload_tx_cfg_nav5.mask       = UBX_TX_CFG_NAV5_MASK;
    _buf.payload_tx_cfg_nav5.dynModel   = UBX_TX_CFG_NAV5_DYNMODEL;
    _buf.payload_tx_cfg_nav5.fixMode    = UBX_TX_CFG_NAV5_FIXMODE;
    send_message(UBX_MSG_CFG_NAV5, _buf.raw, sizeof(_buf.payload_tx_cfg_nav5));

    if (!wait_for_ack(UBX_MSG_CFG_NAV5, UBX_CONFIG_TIMEOUT)) UBX_WARN("No ack for CFG-NAV5");

    /* configure message rates */
    /* the last argument is divisor for measurement rate (set by CFG RATE) */
    configure_message_rate(UBX_MSG_NAV_PVT, 1);   
    if (!wait_for_ack(UBX_MSG_CFG_MSG, UBX_CONFIG_TIMEOUT)) UBX_WARN("No ack for CFG-MSG NAV-PVT");

    configure_message_rate(UBX_MSG_NAV_STATUS, 1);
    if (!wait_for_ack(UBX_MSG_CFG_MSG, UBX_CONFIG_TIMEOUT)) UBX_WARN("No ack for CFG-MSG NAV-STATUS");

    // Save configuration to battery-backed RAM
    memset(&_buf.payload_tx_cfg_cfg, 0, sizeof(_buf.payload_tx_cfg_cfg));
    _buf.payload_tx_cfg_cfg.saveMask = 0x0000000B; // save ioPort, msgConf, and navConf
    _buf.payload_tx_cfg_cfg.deviceMask = 0x01; // save to BBR
    send_message(UBX_MSG_CFG_CFG, _buf.raw, sizeof(_buf.payload_tx_cfg_cfg));
    if (wait_for_ack(UBX_MSG_CFG_CFG, UBX_CONFIG_TIMEOUT)) {
      return true;
    } else {
      UBX_WARN("No ack for CFG-CFG");
      return false;
    }
}

void UBX_NEOM8N::update() {
    while (serial->available() > 0) {
		parse_char(serial->read());
	}
}

void UBX_NEOM8N::stop() {
    // Reset the AssistNow Autonomous computation status
    _aop_status = true;
    // Wait until AssistNow Autonomous computations are done
    configure_message_rate(UBX_MSG_NAV_AOPSTATUS, 1);
    if (!wait_for_ack(UBX_MSG_CFG_MSG, UBX_CONFIG_TIMEOUT)) {
        UBX_WARN("No ack for CFG-MSG NAV-AOPSTATUS");
    }

    while (_aop_status) {
        UBX_TRACE_CONFIG("[%lu] Waiting for AOP", millis());
        receive(100);
    }

    // Perform a clean shutdown of the receiver by requesting it to back up navigation data to
    // battery-backed RAM and shut down
    memset(&_buf.payload_tx_rxm_pmreq, 0, sizeof(_buf.payload_tx_rxm_pmreq));
    _buf.payload_tx_rxm_pmreq.duration = 0; // infinite duration (until next power on)
    _buf.payload_tx_rxm_pmreq.flags    = 0x02; // go into backup mode

    send_message(UBX_MSG_RXM_PMREQ, _buf.raw, sizeof(_buf.payload_tx_rxm_pmreq));

    // Wait for silence
    wait_for_silence(100);
}

void UBX_NEOM8N::cold_start() {
    memset(&_buf.payload_tx_cfg_rst, 0, sizeof(_buf.payload_tx_cfg_rst));
    _buf.payload_tx_cfg_rst.navBbrMask = 0xFF;
    _buf.payload_tx_cfg_rst.resetMode = 0x02;

    send_message(UBX_MSG_CFG_RST, _buf.raw, sizeof(_buf.payload_tx_cfg_rst));
}

void UBX_NEOM8N::assist() {
    if (!Particle.connected()) return;

    if (_last_assistnow_fetch_unixtime == 0 || Time.now() > _last_assistnow_fetch_unixtime + ASSISTNOW_FETCH_INTERVAL_S) {
        UBX_TRACE_ASSIST("[%lu] Requesting AssistNow Online.", millis());

        if (!_assistnow_ip) {
            UBX_TRACE_ASSIST("[%lu] Resolving AssistNow domain.", millis());
            #if ( PLATFORM_ID == PLATFORM_ELECTRON_PRODUCTION)
            // electron
            _assistnow_ip = Cellular.resolve(ASSISTNOW_DOMAIN);
            #elif (PLATFORM_ID == PLATFORM_PHOTON_PRODUCTION || PLATFORM_ID == 88)
            // photon or redbear's duo
            _assistnow_ip = WiFi.resolve(ASSISTNOW_DOMAIN);
            #endif
        }

        if (_assistnow_ip) {
            TCPClient client;
            UBX_TRACE_ASSIST("[%lu] Connecting to AssistNow.", millis());
            if (client.connect(_assistnow_ip, 80)) {
                client.printf(
                    "GET /GetOnlineData.ashx?token=%s;datatype=eph,alm,aux; HTTP/1.1\n",ASSISTNOW_TOKEN);
                client.printf("Host: %s\n\n", ASSISTNOW_DOMAIN);

                UBX_TRACE_ASSIST("[%lu] Sent request:", millis());
                UBX_TRACE_ASSIST("[%lu] GET /GetOnlineData.ashx?"
                                 "token=%s;datatype=eph,alm,aux; HTTP/1.1",
                                 millis(), ASSISTNOW_TOKEN);
                UBX_TRACE_ASSIST("[%lu] Host: %s", millis(), ASSISTNOW_DOMAIN);

                UBX_TRACE_ASSIST("[%lu] Waiting for AssistNow response.", millis());
                uint32_t last_received = millis();
                const uint32_t timeout = 10000;
                uint32_t response_size = 0;
                // Find HTTP response body (all content after the first double newline) and forward
                // it to the receiver
                uint8_t num_newlines_seen = 0;
                while (client.connected() && millis() < last_received + timeout) {
                    while (client.available() > 0) {
                        last_received = millis();
                        char c = client.read();

                        if (num_newlines_seen != 2) {
                            Serial.write(c);
                        }

                        if (num_newlines_seen == 2) {
                            serial->write(c);
                            ++response_size;
                        } else if (c == '\n') {
                            ++num_newlines_seen;
                        } else if (c != '\r') {
                            num_newlines_seen = 0;
                        }

                        // Process messages from receiver to avoid dropping position fixes
                        update();
                    }
                }
                client.stop();
                UBX_TRACE_ASSIST("[%lu] Forwarded %lu bytes of AssistNow to receiver.", millis(), response_size);

                _last_assistnow_fetch_unixtime = Time.now();
            } else {
                UBX_TRACE_ASSIST("[%lu] Failed to connect to AssistNow.", millis());
            }
        } else {
            UBX_TRACE_ASSIST("[%lu] Failed to resolve AssistNow domain.", millis());
        }
    }
    /*else {
        UBX_TRACE_ASSIST("[%lu] Skipping AssistNow fetch; %d seconds until next fetch.", millis(),
                         _last_assistnow_fetch_unixtime + ASSISTNOW_FETCH_INTERVAL_S - Time.now());
    }*/
}

bool UBX_NEOM8N::wait_for_ack(const uint16_t msg, const uint32_t timeout) {
    _ack_state = UBX_ACK_WAITING;
    _ack_waiting_msg = msg; // memorize sent msg class&ID for ACK check

    uint32_t time_started = millis();

    while ((_ack_state == UBX_ACK_WAITING) && (millis() < time_started + timeout)) {
        receive(timeout);
    }

    bool ret = (_ack_state == UBX_ACK_GOT_ACK);
    _ack_state = UBX_ACK_IDLE;
    return ret;
}

void UBX_NEOM8N::receive(const uint32_t max_timeout) {
    uint32_t timeout = max_timeout;

    uint32_t last_received = millis();
    while (millis() < last_received + timeout) {
        while (serial->available() > 0) {
            last_received = millis();
            timeout = UBX_PACKET_TIMEOUT;
            parse_char(serial->read());
        }
    }
}

void UBX_NEOM8N::wait_for_silence(const uint32_t timeout) {
    uint32_t last_received = millis();
    while (millis() < last_received + timeout) {
        while (serial->available() > 0) {
            last_received = millis();
            parse_char(serial->read());
        }
    }
}

void UBX_NEOM8N::parse_char(const uint8_t b) {
    switch (_decode_state) {
    /* Expecting Sync1 */
    case UBX_DECODE_SYNC1:
        if (b == UBX_SYNC1) {   // Sync1 found --> expecting Sync2
            UBX_TRACE_PARSER("A");
            _decode_state = UBX_DECODE_SYNC2;
        }
        break;

    /* Expecting Sync2 */
    case UBX_DECODE_SYNC2:
        if (b == UBX_SYNC2) {   // Sync2 found --> expecting Class
            UBX_TRACE_PARSER("B");
            _decode_state = UBX_DECODE_CLASS;
        } else {        // Sync1 not followed by Sync2: reset parser
            decode_init();
        }
        break;

    /* Expecting Class */
    case UBX_DECODE_CLASS:
        UBX_TRACE_PARSER("C");
        add_byte_to_checksum(b);  // checksum is calculated for everything except Sync and Checksum bytes
        _rx_msg = b;
        _decode_state = UBX_DECODE_ID;
        break;

    /* Expecting ID */
    case UBX_DECODE_ID:
        UBX_TRACE_PARSER("D");
        add_byte_to_checksum(b);
        _rx_msg |= b << 8;
        _decode_state = UBX_DECODE_LENGTH1;
        break;

    /* Expecting first length byte */
    case UBX_DECODE_LENGTH1:
        UBX_TRACE_PARSER("E");
        add_byte_to_checksum(b);
        _rx_payload_length = b;
        _decode_state = UBX_DECODE_LENGTH2;
        break;

    /* Expecting second length byte */
    case UBX_DECODE_LENGTH2:
        UBX_TRACE_PARSER("F");
        add_byte_to_checksum(b);
        _rx_payload_length |= b << 8;   // calculate payload size

        if (payload_rx_init()) {   // start payload reception
            _decode_state = (_rx_payload_length > 0) ? UBX_DECODE_PAYLOAD : UBX_DECODE_CHKSUM1;
        } else {
            // payload will not be handled, discard message
            decode_init();
        }
        break;

    /* Expecting payload */
    case UBX_DECODE_PAYLOAD:
        UBX_TRACE_PARSER(".");
        add_byte_to_checksum(b);

        if (payload_rx_add(b)) {
            // payload complete, expecting checksum
            _decode_state = UBX_DECODE_CHKSUM1;
        } else {
            // expecting more payload, stay in state UBX_DECODE_PAYLOAD
        }
        break;

    /* Expecting first checksum byte */
    case UBX_DECODE_CHKSUM1:
        if (_rx_ck_a != b) {
            UBX_WARN("ubx checksum err for msg %0.4X", _rx_msg);
            decode_init();
        } else {
            _decode_state = UBX_DECODE_CHKSUM2;
        }
        break;

    /* Expecting second checksum byte */
    case UBX_DECODE_CHKSUM2:
        if (_rx_ck_b != b) {
            UBX_WARN("ubx checksum err for msg %0.4X", _rx_msg);
        } else {
            payload_rx_done();    // finish payload processing
        }

        decode_init();
        break;
    }
}

bool UBX_NEOM8N::payload_rx_init() {
    switch (_rx_msg) {
    case UBX_MSG_NAV_PVT:
        if (_rx_payload_length != sizeof(ubx_payload_rx_nav_pvt_t)) {
            _rx_state = UBX_RXMSG_ERROR_LENGTH;
        } else {
            _rx_state = UBX_RXMSG_HANDLE;
        }
        break;

    case UBX_MSG_NAV_STATUS:
        if (_rx_payload_length != sizeof(ubx_payload_rx_nav_status_t)) {
            _rx_state = UBX_RXMSG_ERROR_LENGTH;
        } else {
            _rx_state = UBX_RXMSG_HANDLE;
        }
        break;

    case UBX_MSG_NAV_AOPSTATUS:
        if (_rx_payload_length != sizeof(ubx_payload_rx_nav_aopstatus_t)) {
            _rx_state = UBX_RXMSG_ERROR_LENGTH;
        } else {
            _rx_state = UBX_RXMSG_HANDLE;
        }
        break;

    case UBX_MSG_ACK_ACK:
        if (_rx_payload_length != sizeof(ubx_payload_rx_ack_ack_t)) {
            _rx_state = UBX_RXMSG_ERROR_LENGTH;
        } else {
            _rx_state = UBX_RXMSG_HANDLE;
        }

        break;

    case UBX_MSG_ACK_NAK:
        if (_rx_payload_length != sizeof(ubx_payload_rx_ack_nak_t)) {
            _rx_state = UBX_RXMSG_ERROR_LENGTH;
        } else {
            _rx_state = UBX_RXMSG_HANDLE;
        }
        break;

    default:
        _rx_state = UBX_RXMSG_DISABLE;  // disable all other messages
        break;
    }

    switch (_rx_state) {
    case UBX_RXMSG_HANDLE:  // handle message
    case UBX_RXMSG_IGNORE:  // ignore message but don't report error
        return true;

    case UBX_RXMSG_DISABLE: // disable unexpected messages
        return false;

    case UBX_RXMSG_ERROR_LENGTH:    // error: invalid length
        UBX_WARN("ubx msg invalid len");
        return false;

    default:    // invalid message state
        UBX_WARN("ubx internal err1");
        return false;
    }
}

bool UBX_NEOM8N::payload_rx_add(const uint8_t b) {
    _buf.raw[_rx_payload_index] = b;
    return (++_rx_payload_index >= _rx_payload_length);
}

void UBX_NEOM8N::payload_rx_done() {
    // return if no message handled
    if (_rx_state != UBX_RXMSG_HANDLE) {
        return;
    }

    // handle message
    switch (_rx_msg) {
    case UBX_MSG_NAV_PVT:
        //uncomment to enable log entry for every nav-pvt msg received
        UBX_TRACE_RXMSG("[%lu] Rx NAV-PVT", millis());
        _callback(_rx_msg, _buf);
        break;

    case UBX_MSG_NAV_STATUS:
        //uncomment to enable log entry for every nav-status msg received
        UBX_TRACE_RXMSG("[%lu] Rx NAV-STATUS", millis());
        _callback(_rx_msg, _buf);
        break;

    case UBX_MSG_NAV_AOPSTATUS:
        UBX_TRACE_RXMSG("[%lu] Rx NAV-AOPSTATUS %d", millis(), _buf.payload_rx_nav_aopstatus.status);
        _aop_status = (_buf.payload_rx_nav_aopstatus.status != 0);
        break;

    case UBX_MSG_ACK_ACK:
        UBX_TRACE_RXMSG("[%lu] Rx ACK-ACK", millis());

        if ((_ack_state == UBX_ACK_WAITING) && (_buf.payload_rx_ack_ack.msg == _ack_waiting_msg)) {
            _ack_state = UBX_ACK_GOT_ACK;
        }
        break;

    case UBX_MSG_ACK_NAK:
        UBX_TRACE_RXMSG("[%lu] Rx ACK-NAK", millis());

        if ((_ack_state == UBX_ACK_WAITING) && (_buf.payload_rx_ack_ack.msg == _ack_waiting_msg)) {
            _ack_state = UBX_ACK_GOT_NAK;
        }
        break;
    }
}

void UBX_NEOM8N::decode_init() {
    _decode_state = UBX_DECODE_SYNC1;
    _rx_ck_a = 0;
    _rx_ck_b = 0;
    _rx_payload_length = 0;
    _rx_payload_index = 0;
}

void UBX_NEOM8N::add_byte_to_checksum(const uint8_t b) {
    _rx_ck_a = _rx_ck_a + b;
    _rx_ck_b = _rx_ck_b + _rx_ck_a;
}

void UBX_NEOM8N::calc_checksum(const uint8_t *buffer, const uint16_t length, ubx_checksum_t *checksum) {
    for (uint16_t i = 0; i < length; i++) {
        checksum->ck_a = checksum->ck_a + buffer[i];
        checksum->ck_b = checksum->ck_b + checksum->ck_a;
    }
}

void UBX_NEOM8N::configure_message_rate(const uint16_t msg, const uint8_t rate) {
    ubx_payload_tx_cfg_msg_t cfg_msg;   // don't use _buf (allow interleaved operation)

    cfg_msg.msg  = msg;
    cfg_msg.rate = rate;

    send_message(UBX_MSG_CFG_MSG, (uint8_t *)&cfg_msg, sizeof(cfg_msg));
}

void UBX_NEOM8N::send_message(const uint16_t msg, const uint8_t *payload, const uint16_t length) {
    ubx_header_t   header = {UBX_SYNC1, UBX_SYNC2};
    ubx_checksum_t checksum = {0, 0};

    // Populate header
    header.msg    = msg;
    header.length = length;

    // Calculate checksum
    calc_checksum(reinterpret_cast<const uint8_t *>(&header) + 2, sizeof(header) - 2, &checksum); // skip 2 sync bytes

    if (payload != nullptr) {
        calc_checksum(payload, length, &checksum);
    }

    // Send message
    serial->write(reinterpret_cast<const uint8_t *>(&header), sizeof(header));

    if (payload != nullptr) {
        serial->write(payload, length);
    }

    serial->write(reinterpret_cast<const uint8_t *>(&checksum), sizeof(checksum));
}
