#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "fc/fc_msp_box.h"
#include "fc/rc_modes.h"
#include "io/serial.h"
#include "io/gps.h"
#include "flight/failsafe.h"
#include "flight/imu.h"
#include "navigation/navigation.h"
#include "sensors/battery.h"

void targetConfiguration(void)
{
    // ── штатное для платы (не трогаем) ──
    pinioBoxConfigMutable()->permanentId[0] = BOX_PERMANENT_ID_USER1;
    serialConfigMutable()->portConfigs[findSerialPortIndexByIdentifier(SERIAL_PORT_USART4)].functionMask = FUNCTION_MSP;

    // ── наши порты: UART1 приёмник, UART5 телеметрия ESC, UART6 GPS ──
    serialConfigMutable()->portConfigs[findSerialPortIndexByIdentifier(SERIAL_PORT_USART2)].functionMask = 0;
    serialConfigMutable()->portConfigs[findSerialPortIndexByIdentifier(SERIAL_PORT_USART1)].functionMask = FUNCTION_RX_SERIAL;
    serialConfigMutable()->portConfigs[findSerialPortIndexByIdentifier(SERIAL_PORT_USART5)].functionMask = FUNCTION_ESC_SENSOR;
    serialConfigMutable()->portConfigs[findSerialPortIndexByIdentifier(SERIAL_PORT_USART6)].functionMask = FUNCTION_GPS;

    // ── эталон висения ──
    positionEstimationConfigMutable()->default_alt_sensor = 0;          // inav_default_alt_sensor = GPS
    navConfigMutable()->mc.althold_throttle_type = MC_ALT_HOLD_HOVER;   // газ висения из hover_thr
    navConfigMutable()->general.flags.disarm_on_landing = 1;
    navConfigMutable()->general.flags.extra_arming_safety = NAV_EXTRA_ARMING_SAFETY_ON;
    navConfigMutable()->general.land_detect_sensitivity = 5;
    navConfigMutable()->general.emerg_descent_rate = 300;               // аварийная посадка 3 м/с

    batteryProfilesMutable(0)->nav.mc.hover_throttle = 1420;            // газ висения, профиль 1
    batteryProfilesMutable(0)->failsafe_throttle = 1000;

    failsafeConfigMutable()->failsafe_procedure = FAILSAFE_PROCEDURE_AUTO_LANDING;
    failsafeConfigMutable()->failsafe_delay = 10;
    failsafeConfigMutable()->failsafe_off_delay = 0;
    failsafeConfigMutable()->failsafe_min_distance = 0;
    failsafeConfigMutable()->failsafe_min_distance_procedure = FAILSAFE_PROCEDURE_AUTO_LANDING;

    imuConfigMutable()->small_angle = 25;

    gpsConfigMutable()->ubloxUseGlonass = true;
    gpsConfigMutable()->gpsMinSats = 8;

    // ── тумблеры ──
    // CH5 верх = ARM
    modeActivationConditionsMutable(0)->modeId = BOXARM;
    modeActivationConditionsMutable(0)->auxChannelIndex = 0;
    modeActivationConditionsMutable(0)->range.startStep = CHANNEL_VALUE_TO_STEP(1700);
    modeActivationConditionsMutable(0)->range.endStep   = CHANNEL_VALUE_TO_STEP(2100);
    // CH8 низ = ANGLE
    modeActivationConditionsMutable(1)->modeId = BOXANGLE;
    modeActivationConditionsMutable(1)->auxChannelIndex = 3;
    modeActivationConditionsMutable(1)->range.startStep = CHANNEL_VALUE_TO_STEP(900);
    modeActivationConditionsMutable(1)->range.endStep   = CHANNEL_VALUE_TO_STEP(1300);
    // CH8 середина = HORIZON
    modeActivationConditionsMutable(2)->modeId = BOXHORIZON;
    modeActivationConditionsMutable(2)->auxChannelIndex = 3;
    modeActivationConditionsMutable(2)->range.startStep = CHANNEL_VALUE_TO_STEP(1300);
    modeActivationConditionsMutable(2)->range.endStep   = CHANNEL_VALUE_TO_STEP(1700);
    // CH8 середина+верх = ALTHOLD
    modeActivationConditionsMutable(3)->modeId = BOXNAVALTHOLD;
    modeActivationConditionsMutable(3)->auxChannelIndex = 3;
    modeActivationConditionsMutable(3)->range.startStep = CHANNEL_VALUE_TO_STEP(1300);
    modeActivationConditionsMutable(3)->range.endStep   = CHANNEL_VALUE_TO_STEP(2100);
    // CH8 верх = POSHOLD
    modeActivationConditionsMutable(4)->modeId = BOXNAVPOSHOLD;
    modeActivationConditionsMutable(4)->auxChannelIndex = 3;
    modeActivationConditionsMutable(4)->range.startStep = CHANNEL_VALUE_TO_STEP(1700);
    modeActivationConditionsMutable(4)->range.endStep   = CHANNEL_VALUE_TO_STEP(2100);
}
