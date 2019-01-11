/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file ubx.h
 *
 * U-Blox protocol definition. Following u-blox 6/7/8 Receiver Description
 * including Prototol Specification.
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 *
 * @author Hannes Delago
 *   (rework, add ubx7+ compatibility)
 *
 * @author Ankur Dave
 *   (port to Particle Electron, specialize for ubx 8, simplify, add commands)
 *    https://github.com/ankurdave/IntervalTracker
 * @author Jaafar Benabdallah
 *   (port to Particle Photon/Redbear Duo, NeoM8N module)
 *
 *s
 */

//#include <functional>
#include <stdint.h>

#ifndef UBX_NEOM8N_H_
#define UBX_NEOM8N_H_

#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62

/* Message Classes */
#define UBX_CLASS_ACK       0x05
#define UBX_CLASS_CFG       0x06
#define UBX_CLASS_MGA       0x13
#define UBX_CLASS_MON       0x0A
#define UBX_CLASS_NAV       0x01
#define UBX_CLASS_RXM       0x02

/* Message IDs */
#define UBX_ID_ACK_ACK          0x01
#define UBX_ID_ACK_NAK          0x00
#define UBX_ID_CFG_CFG          0x09
#define UBX_ID_CFG_MSG          0x01
#define UBX_ID_CFG_NAV5         0x24
#define UBX_ID_CFG_NAVX5        0x23
#define UBX_ID_CFG_PRT          0x00
#define UBX_ID_CFG_PM2          0x3B
#define UBX_ID_CFG_RATE         0x08
#define UBX_ID_CFG_RST          0x04
#define UBX_ID_CFG_SBAS         0x16
#define UBX_ID_MGA_DBD          0x80
#define UBX_ID_MGA_INI_TIME_UTC 0x40
#define UBX_ID_MON_HW           0x09
#define UBX_ID_NAV_AOPSTATUS    0x60
#define UBX_ID_NAV_PVT          0x07
#define UBX_ID_NAV_STATUS       0x03
#define UBX_ID_RXM_PMREQ        0x41

/* Message Classes & IDs */
#define UBX_MSG_ACK_ACK          ((UBX_CLASS_ACK) | UBX_ID_ACK_ACK << 8)
#define UBX_MSG_ACK_NAK          ((UBX_CLASS_ACK) | UBX_ID_ACK_NAK << 8)
#define UBX_MSG_CFG_CFG          ((UBX_CLASS_CFG) | UBX_ID_CFG_CFG << 8)
#define UBX_MSG_CFG_MSG          ((UBX_CLASS_CFG) | UBX_ID_CFG_MSG << 8)
#define UBX_MSG_CFG_NAV5         ((UBX_CLASS_CFG) | UBX_ID_CFG_NAV5 << 8)
#define UBX_MSG_CFG_NAVX5        ((UBX_CLASS_CFG) | UBX_ID_CFG_NAVX5 << 8)
#define UBX_MSG_CFG_PRT          ((UBX_CLASS_CFG) | UBX_ID_CFG_PRT << 8)
#define UBX_MSG_CFG_PM2          ((UBX_CLASS_CFG) | UBX_ID_CFG_PM2 << 8)
#define UBX_MSG_CFG_RATE         ((UBX_CLASS_CFG) | UBX_ID_CFG_RATE << 8)
#define UBX_MSG_CFG_RST          ((UBX_CLASS_CFG) | UBX_ID_CFG_RST << 8)
#define UBX_MSG_CFG_SBAS         ((UBX_CLASS_CFG) | UBX_ID_CFG_SBAS << 8)
#define UBX_MSG_MGA_DBD          ((UBX_CLASS_MGA) | UBX_ID_MGA_DBD << 8)
#define UBX_MSG_MGA_INI_TIME_UTC ((UBX_CLASS_MGA) | UBX_ID_MGA_INI_TIME_UTC << 8)
#define UBX_MSG_MON_HW           ((UBX_CLASS_MON) | UBX_ID_MON_HW << 8)
#define UBX_MSG_NAV_AOPSTATUS    ((UBX_CLASS_NAV) | UBX_ID_NAV_AOPSTATUS << 8)
#define UBX_MSG_NAV_PVT          ((UBX_CLASS_NAV) | UBX_ID_NAV_PVT << 8)
#define UBX_MSG_NAV_STATUS       ((UBX_CLASS_NAV) | UBX_ID_NAV_STATUS << 8)
#define UBX_MSG_RXM_PMREQ        ((UBX_CLASS_RXM) | UBX_ID_RXM_PMREQ << 8)

/* RX NAV-PVT message content details */
/*   Bitfield "valid" masks */
#define UBX_RX_NAV_PVT_VALID_VALIDDATE      0x01    /** validDate (Valid UTC Date) */
#define UBX_RX_NAV_PVT_VALID_VALIDTIME      0x02    /** validTime (Valid UTC Time) */
#define UBX_RX_NAV_PVT_VALID_FULLYRESOLVED  0x04    /** fullyResolved (1 = UTC Time of Day has been fully resolved (no seconds uncertainty)) */

/*   Bitfield "flags" masks */
#define UBX_RX_NAV_PVT_FLAGS_GNSSFIXOK      0x01    /** gnssFixOK (A valid fix (i.e within DOP & accuracy masks)) */
#define UBX_RX_NAV_PVT_FLAGS_DIFFSOLN       0x02    /** diffSoln (1 if differential corrections were applied) */
#define UBX_RX_NAV_PVT_FLAGS_PSMSTATE       0x1C    /** psmState (Power Save Mode state (see Power Management)) */
#define UBX_RX_NAV_PVT_FLAGS_HEADVEHVALID   0x20    /** headVehValid (Heading of vehicle is valid) */

/* TX CFG-PRT message contents */
#define UBX_TX_CFG_PRT_PORTID         0x01        /** UART1 */
#define UBX_TX_CFG_PRT_MODE           0x000008D0  /** 0b0000100011010000: 8N1 */
#define UBX_TX_CFG_PRT_BAUDRATE       9600
#define UBX_TX_CFG_PRT_INPROTOMASK    0x01        /** UBX in */
#define UBX_TX_CFG_PRT_OUTPROTOMASK   0x03        /** 0x01: UBX + 0x02: NMEA out */

/* TX CFG-RATE message contents */
#define UBX_TX_CFG_RATE_MEASINTERVAL   1000
#define UBX_TX_CFG_RATE_NAVRATE        1
#define UBX_TX_CFG_RATE_TIMEREF        0       /** 0: UTC, 1: GPS time */

/* TX CFG-NAV5 message contents */
#define UBX_TX_CFG_NAV5_MASK       0x0005 /** Only update dynamic model and position fix mode */
#define UBX_TX_CFG_NAV5_DYNMODEL   0      /**
                                           * 0 Portable, 2 Stationary, 3 Pedestrian, 4 Automotive,
                                           * 5 Sea, 6 Airborne <1g, 7 Airborne <2g, 8 Airborne
                                           * <4g, 9 wrist worn
                                           */
#define UBX_TX_CFG_NAV5_FIXMODE    3      /** 1: 2D only, 2: 3D only, 3: Auto 2D/3D */

// AssistNow
static const char ASSISTNOW_DOMAIN[] = "online-live1.services.u-blox.com";
static const int32_t ASSISTNOW_FETCH_INTERVAL_S = 30;

/*** u-blox protocol binary message and payload definitions ***/
#pragma pack(push, 1)

/* General: Header */
typedef struct {
    uint8_t     sync1;
    uint8_t     sync2;
    uint16_t    msg;
    uint16_t    length;
} ubx_header_t;

/* General: Checksum */
typedef struct {
    uint8_t     ck_a;
    uint8_t     ck_b;
} ubx_checksum_t ;

/* Rx NAV-STATUS */
typedef struct {
    uint32_t    iTOW;       /** GPS Time of Week [ms] */
    uint8_t     gpsFix;     /** GPSfix type: 0 = No fix, 1 = Dead Reckoning only, 2 = 2D fix, 3 = 3d-fix, 4 = GPS + dead reckoning, 5 = time only fix */
    uint8_t     flags;
    uint8_t     fixStat;
    uint8_t     flags2;
    uint32_t    ttff;       /** Time to first fix [ms] */
    uint32_t    msss;       /** Milliseconds since startup [ms] */
} ubx_payload_rx_nav_status_t;

/* Rx NAV-PVT (ubx8) */
typedef struct {
    uint32_t    iTOW;       /** GPS Time of Week [ms] */
    uint16_t    year;       /** Year (UTC)*/
    uint8_t     month;      /** Month, range 1..12 (UTC) */
    uint8_t     day;        /** Day of month, range 1..31 (UTC) */
    uint8_t     hour;       /** Hour of day, range 0..23 (UTC) */
    uint8_t     min;        /** Minute of hour, range 0..59 (UTC) */
    uint8_t     sec;        /** Seconds of minute, range 0..60 (UTC) */
    uint8_t     valid;      /** Validity flags (see UBX_RX_NAV_PVT_VALID_...) */
    uint32_t    tAcc;       /** Time accuracy estimate (UTC) [ns] */
    int32_t     nano;       /** Fraction of second (UTC) [-1e9...1e9 ns] */
    uint8_t     fixType;    /** GNSSfix type: 0 = No fix, 1 = Dead Reckoning only, 2 = 2D fix, 3 = 3d-fix, 4 = GNSS + dead reckoning, 5 = time only fix */
    uint8_t     flags;      /** Fix Status Flags (see UBX_RX_NAV_PVT_FLAGS_...) */
    uint8_t     reserved1;
    uint8_t     numSV;      /** Number of SVs used in Nav Solution */
    int32_t     lon;        /** Longitude [1e-7 deg] */
    int32_t     lat;        /** Latitude [1e-7 deg] */
    int32_t     height;     /** Height above ellipsoid [mm] */
    int32_t     hMSL;       /** Height above mean sea level [mm] */
    uint32_t    hAcc;       /** Horizontal accuracy estimate [mm] */
    uint32_t    vAcc;       /** Vertical accuracy estimate [mm] */
    int32_t     velN;       /** NED north velocity [mm/s]*/
    int32_t     velE;       /** NED east velocity [mm/s]*/
    int32_t     velD;       /** NED down velocity [mm/s]*/
    int32_t     gSpeed;     /** Ground Speed (2-D) [mm/s] */
    int32_t     headMot;    /** Heading of motion (2-D) [1e-5 deg] */
    uint32_t    sAcc;       /** Speed accuracy estimate [mm/s] */
    uint32_t    headAcc;    /** Heading accuracy estimate (motion and vehicle) [1e-5 deg] */
    uint16_t    pDOP;       /** Position DOP [0.01] */
    uint16_t    reserved2;
    uint32_t    reserved3;
    int32_t     headVeh;    /** (ubx8+ only) Heading of vehicle (2-D) [1e-5 deg] */
    uint32_t    reserved4;  /** (ubx8+ only) */
} ubx_payload_rx_nav_pvt_t;

/* Rx NAV-AOPSTATUS */
typedef struct {
    uint32_t iTOW;
    uint8_t aopCfg;
    uint8_t status;
    uint8_t reserved1[10];
} ubx_payload_rx_nav_aopstatus_t;

/* Rx MON-HW (ubx7+) */
typedef struct {
    uint32_t    pinSel;
    uint32_t    pinBank;
    uint32_t    pinDir;
    uint32_t    pinVal;
    uint16_t    noisePerMS;
    uint16_t    agcCnt;
    uint8_t     aStatus;
    uint8_t     aPower;
    uint8_t     flags;
    uint8_t     reserved1;
    uint32_t    usedMask;
    uint8_t     VP[17];
    uint8_t     jamInd;
    uint16_t    reserved3;
    uint32_t    pinIrq;
    uint32_t    pullH;
    uint32_t    pullL;
} ubx_payload_rx_mon_hw_ubx7_t;

/* Rx ACK-ACK */
typedef union {
    uint16_t    msg;
    struct {
        uint8_t clsID;
        uint8_t msgID;
    };
} ubx_payload_rx_ack_ack_t;

/* Rx ACK-NAK */
typedef union {
    uint16_t    msg;
    struct {
        uint8_t clsID;
        uint8_t msgID;
    };
} ubx_payload_rx_ack_nak_t;

/* Tx CFG-PRT */
typedef struct {
    uint8_t     portID;
    uint8_t     reserved0;
    uint16_t    txReady;
    uint32_t    mode;
    uint32_t    baudRate;
    uint16_t    inProtoMask;
    uint16_t    outProtoMask;
    uint16_t    flags;
    uint16_t    reserved5;
} ubx_payload_tx_cfg_prt_t;

/* Tx CFG-PM2 */
typedef struct {
    uint8_t version; /* 0x02 */
    uint8_t reserved1;
    uint8_t maxStartupStateDur;
    uint8_t reserved2;
    uint32_t flags; /* default for SPG 3.01: 0x00021000 (updateEPH=1, mode=1) */
    uint32_t updatePeriod; /* default for SPG 3.01: 1000 */
    uint32_t searchPeriod; /* default for SPG 3.01: 10000 */
    uint32_t gridOffset;
    uint16_t onTime;
    uint16_t minAcqTime;
    uint8_t reserved3[20];
    uint32_t extIntInactivityMs;
} ubx_payload_tx_cfg_pm2_t;

/* Tx CFG-RATE */
typedef struct {
    uint16_t    measRate;   /** Measurement Rate, GPS measurements are taken every measRate milliseconds */
    uint16_t    navRate;    /** Navigation Rate, in number of measurement cycles. This parameter cannot be changed, and must be set to 1 */
    uint16_t    timeRef;    /** Alignment to reference time: 0 = UTC time, 1 = GPS time */
} ubx_payload_tx_cfg_rate_t;

/* Tx CFG-CFG */
typedef struct {
    uint32_t clearMask;
    uint32_t saveMask;
    uint32_t loadMask;
    uint8_t deviceMask;
} ubx_payload_tx_cfg_cfg_t;

/* Tx CFG-RST */
typedef struct {
    uint16_t navBbrMask;
    uint8_t  resetMode;
    uint8_t  reserved1;
} ubx_payload_tx_cfg_rst_t;

/* Tx CFG-NAVX5 */
typedef struct {
    uint16_t version; /** Must be 2 */
    uint16_t mask1;
    uint32_t mask2;
    uint16_t reserved1;
    uint8_t minSVs;
    uint8_t maxSVs;
    uint8_t minCNO;
    uint8_t reserved2;
    uint8_t iniFix3D;
    uint8_t reserved3[2];
    uint8_t ackAiding;
    uint16_t wknRollover;
    uint8_t sigAttenCompMod;
    uint8_t reserved4;
    uint16_t reserved5;
    uint16_t reserved6;
    uint8_t usePPP;
    uint8_t aopCfg;
    uint16_t reserved7;
    uint16_t aopOrbMaxErr;
    uint32_t reserved8;
    uint8_t reserved9[3];
    uint8_t useAdr;
} ubx_payload_tx_cfg_navx5_t;

/* Tx CFG-NAV5 */
typedef struct {
    uint16_t    mask;
    uint8_t     dynModel;   /** Dynamic Platform model: 0 Portable, 2 Stationary, 3 Pedestrian, 4 Automotive, 5 Sea, 6 Airborne <1g, 7 Airborne <2g, 8 Airborne <4g */
    uint8_t     fixMode;    /** Position Fixing Mode: 1 2D only, 2 3D only, 3 Auto 2D/3D */
    int32_t     fixedAlt;
    uint32_t    fixedAltVar;
    int8_t      minElev;
    uint8_t     drLimit;
    uint16_t    pDop;
    uint16_t    tDop;
    uint16_t    pAcc;
    uint16_t    tAcc;
    uint8_t     staticHoldThresh;
    uint8_t     dgpsTimeOut;
    uint8_t     cnoThreshNumSVs;    /** (ubx7+ only, else 0) */
    uint8_t     cnoThresh;      /** (ubx7+ only, else 0) */
    uint16_t    reserved;
    uint16_t    staticHoldMaxDist;  /** (ubx8+ only, else 0) */
    uint8_t     utcStandard;        /** (ubx8+ only, else 0) */
    uint8_t     reserved3;
    uint32_t    reserved4;
} ubx_payload_tx_cfg_nav5_t;

/* tx cfg-sbas */
typedef struct {
    uint8_t     mode;
    uint8_t     usage;
    uint8_t     maxSBAS;
    uint8_t     scanmode2;
    uint32_t    scanmode1;
} ubx_payload_tx_cfg_sbas_t;

/* Tx CFG-MSG */
typedef struct {
    union {
        uint16_t    msg;
        struct {
            uint8_t msgClass;
            uint8_t msgID;
        };
    };
    uint8_t rate;
} ubx_payload_tx_cfg_msg_t;

/* Tx RXM-PMREQ */
typedef struct {
    uint32_t duration;
    uint32_t flags;
} ubx_payload_tx_rxm_pmreq_t;

/* Tx MGA-INI-TIME_UTC */
typedef struct {
    uint8_t type; // 0x10
    uint8_t version; // 0x00
    uint8_t ref; // 0x00 = time applies on receipt of message
    int8_t leapSecs;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t reserved1;
    uint32_t ns;
    uint16_t tAccS;
    uint16_t reserved2;
    uint32_t tAccNs;
} ubx_payload_tx_mga_ini_time_utc_t;

/* General message and payload buffer union */
typedef union {
    ubx_payload_rx_ack_ack_t          payload_rx_ack_ack;
    ubx_payload_rx_ack_nak_t          payload_rx_ack_nak;
    ubx_payload_rx_mon_hw_ubx7_t      payload_rx_mon_hw_ubx7;
    ubx_payload_rx_nav_aopstatus_t    payload_rx_nav_aopstatus;
    ubx_payload_rx_nav_pvt_t          payload_rx_nav_pvt;
    ubx_payload_rx_nav_status_t       payload_rx_nav_status;
    ubx_payload_tx_cfg_cfg_t          payload_tx_cfg_cfg;
    ubx_payload_tx_cfg_msg_t          payload_tx_cfg_msg;
    ubx_payload_tx_cfg_nav5_t         payload_tx_cfg_nav5;
    ubx_payload_tx_cfg_navx5_t        payload_tx_cfg_navx5;
    ubx_payload_tx_cfg_prt_t          payload_tx_cfg_prt;
    ubx_payload_tx_cfg_pm2_t          payload_tx_cfg_pm2;
    ubx_payload_tx_cfg_rate_t         payload_tx_cfg_rate;
    ubx_payload_tx_cfg_rst_t          payload_tx_cfg_rst;
    ubx_payload_tx_cfg_sbas_t         payload_tx_cfg_sbas;
    ubx_payload_tx_mga_ini_time_utc_t payload_tx_mga_ini_time_utc;
    ubx_payload_tx_rxm_pmreq_t        payload_tx_rxm_pmreq;
    // TODO: determine length needed here
    uint8_t                           raw[256];
} ubx_buf_t;

#pragma pack(pop)
/*** END OF u-blox protocol binary message and payload definitions ***/

/* Decoder state */
typedef enum {
    UBX_DECODE_SYNC1 = 0,
    UBX_DECODE_SYNC2,
    UBX_DECODE_CLASS,
    UBX_DECODE_ID,
    UBX_DECODE_LENGTH1,
    UBX_DECODE_LENGTH2,
    UBX_DECODE_PAYLOAD,
    UBX_DECODE_CHKSUM1,
    UBX_DECODE_CHKSUM2
} ubx_decode_state_t;

/* Rx message state */
typedef enum {
    UBX_RXMSG_IGNORE = 0,
    UBX_RXMSG_HANDLE,
    UBX_RXMSG_DISABLE,
    UBX_RXMSG_ERROR_LENGTH
} ubx_rxmsg_state_t;

/* ACK state */
typedef enum {
    UBX_ACK_IDLE = 0,
    UBX_ACK_WAITING,
    UBX_ACK_GOT_ACK,
    UBX_ACK_GOT_NAK
} ubx_ack_state_t;

class UBX_NEOM8N{
public:
    UBX_NEOM8N(std::function<void(uint16_t, const ubx_buf_t &)> callback);
    ~UBX_NEOM8N();

    /**
     * Turn on the u-blox receiver and set configuration. Blocks until the receiver is properly
     * configured.
     */
    bool start(USARTSerial *uart, uint32_t baud, uint8_t en_pin);

    /**
     * Read buffered input data from the u-blox receiver and invoke the callback on new messages.
     * Does not block.
     */
    void update();

    /**
     * Turn off the u-blox receiver. Blocks until the receiver is off.
     */
    void stop();

    /**
     * For testing, clear the navigation data and reset the u-blox receiver, forcing a cold start.
     */
    void cold_start();

    /**
     * If needed, request assistance from the AssistNow server over the cellular network. Blocks
     * until assistance is received.
     */
    void assist();

private:
    /** Parse a single byte from the u-blox receiver. */
    void parse_char(const uint8_t b);

    /**
     * Receive an input packet or the specified number of milliseconds of silence.
     */
    void receive(const uint32_t timeout);

    /**
     * Wait for the specified number of milliseconds of silence.
     */
    void wait_for_silence(const uint32_t timeout);

    /**
     * Wait for message acknowledgement. Return true if an ack for the given message was received.
     */
    bool wait_for_ack(const uint16_t msg, const uint32_t timeout);

    /**
     * Send a message to the UBX receiver. Used for configuration.
     */
    void send_message(const uint16_t msg, const uint8_t *payload, const uint16_t length);

    /**
     * Set the desired fix rate. Used for configuration.
     */
    void configure_message_rate(const uint16_t msg, const uint8_t rate);

    /**
     * Start payload rx. Return true if the message should continue to be parsed, false if parsing
     * should be aborted.
     */
    bool payload_rx_init(void);

    /**
     * Add payload rx byte. Return true on message completion.
     */
    bool payload_rx_add(const uint8_t b);

    /**
     * Finish payload rx.
     */
    void payload_rx_done();

    /**
     * Reset the parse state machine for a fresh start
     */
    void decode_init();

    /**
     * While parsing add every byte (except the sync bytes) to the checksum
     */
    void add_byte_to_checksum(const uint8_t);

    /**
     * Calculate & add checksum for given buffer
     */
    void calc_checksum(const uint8_t *buffer, const uint16_t length, ubx_checksum_t *checksum);

    /* uart peripheral to be used */
    USARTSerial *serial;
    /* power supply shutdown pin (active low) */
    uint8_t enable_v33;

    ubx_ack_state_t    _ack_state;
    ubx_decode_state_t _decode_state;
    uint16_t           _rx_msg;
    ubx_rxmsg_state_t  _rx_state;
    uint16_t           _rx_payload_length;
    uint16_t           _rx_payload_index;
    uint8_t            _rx_ck_a;
    uint8_t            _rx_ck_b;
    uint16_t           _ack_waiting_msg;
    ubx_buf_t          _buf;

    /**
     * Status of AssistNow Autonomous computations, used to determine when to power off the
     * receiver. True if computations are running or the status is unknown; false if the system is
     * known to be idle.
     */
    bool _aop_status;

    int32_t _last_assistnow_fetch_unixtime;
    IPAddress _assistnow_ip;


    std::function<void(uint16_t, const ubx_buf_t &)> _callback;
};

#endif /* UBX_H_ */
