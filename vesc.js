

var START_SHORT = 2;
var START_LONG = 3;
var END_BYTE = 3;

var COMM_FW_VERSION = 0;
var COMM_GET_VALUES = 4;
var COMM_SET_DUTY = 5;
var COMM_SET_CURRENT = 6;
var COMM_SET_CURRENT_BRAKE = 7;
var COMM_SET_RPM = 8;
var COMM_ALIVE = 30;
var COMM_FORWARD_CAN = 34;

var pollAccumulatorMs = 0;
var aliveAccumulatorMs = 0;
var rxBuffer = [];

function init()
{
	pollAccumulatorMs = 0;
	aliveAccumulatorMs = 0;
	rxBuffer = [];
	setStatus("Ready. Configure the Serial module for raw bytes at the VESC UART baud rate.");
}

function scriptParameterChanged(param)
{
	handleModuleTrigger(param);
}

function update(deltaTime)
{
	var deltaMs = deltaTime * 1000.0;

	if (getParamValue("autoPoll", true))
	{
		pollAccumulatorMs += deltaMs;
		if (pollAccumulatorMs >= getParamValue("pollInterval", 250))
		{
			pollAccumulatorMs = 0;
			sendSimpleCommand(COMM_GET_VALUES);
		}
	}
	else
	{
		pollAccumulatorMs = 0;
	}

	if (getParamValue("autoAlive", true))
	{
		aliveAccumulatorMs += deltaMs;
		if (aliveAccumulatorMs >= getParamValue("aliveInterval", 100))
		{
			aliveAccumulatorMs = 0;
			sendSimpleCommand(COMM_ALIVE);
		}
	}
	else
	{
		aliveAccumulatorMs = 0;
	}
}

function moduleParameterChanged(param)
{
	handleModuleTrigger(param);

	if (isRawLoggingEnabled())
	{
		script.log("Module parameter changed: " + param.name);
	}
}

function moduleValueChanged(value)
{
	if (isRawLoggingEnabled())
	{
		script.log("Module value changed: " + value.name);
	}
}

function dataReceived(data)
{
	var incoming = normalizeIncomingBytes(data);
	var i;

	if (incoming.length === 0)
	{
		return;
	}

	if (isRawLoggingEnabled())
	{
		script.log("RX: " + incoming.join(" "));
	}

	for (i = 0; i < incoming.length; i++)
	{
		rxBuffer.push(incoming[i]);
	}

	processRxBuffer();
}

function sendSelectedControl()
{
	var controlMode = normalizeControlMode(getParamValue("controlMode", "Duty"));

	if (controlMode === "duty")
	{
		sendDuty(getParamValue("duty", 0));
		return;
	}

	if (controlMode === "current")
	{
		sendCurrent(getParamValue("current", 0));
		return;
	}

	if (controlMode === "brakecurrent" || controlMode === "brake current")
	{
		sendBrakeCurrent(getParamValue("brakeCurrent", 0));
		return;
	}

	if (controlMode === "rpm")
	{
		sendRpm(getParamValue("rpmCommand", 0));
		return;
	}

	setStatus("Unsupported control mode: " + getParamValue("controlMode", "Duty"));
}

function sendSimpleCommand(command)
{
	if(!local.parameters.isConnected.get()) return;
	var payload = buildPayload(command);
	sendPayload(payload);
	setStatus("Sent command " + commandName(command));
}

function sendDuty(duty)
{
	if(!local.parameters.isConnected.get()) return;
	var payload = buildPayload(COMM_SET_DUTY);
	appendInt32(payload, Math.round(duty * 100000.0));
	sendPayload(payload);
	setStatus("Sent duty command: " + duty);
}

function sendCurrent(current)
{
	if(!local.parameters.isConnected.get()) return;
	var payload = buildPayload(COMM_SET_CURRENT);
	appendInt32(payload, Math.round(current * 1000.0));
	sendPayload(payload);
	setStatus("Sent current command: " + current + " A");
}

function sendBrakeCurrent(current)
{
	if(!local.parameters.isConnected.get()) return;
	var payload = buildPayload(COMM_SET_CURRENT_BRAKE);
	appendInt32(payload, Math.round(current * 1000.0));
	sendPayload(payload);
	setStatus("Sent brake current command: " + current + " A");
}

function sendRpm(rpm)
{
	if(!local.parameters.isConnected.get()) return;
	var payload = buildPayload(COMM_SET_RPM);
	appendInt32(payload, Math.round(rpm));
	sendPayload(payload);
	setStatus("Sent RPM command: " + rpm);
}

function buildPayload(command)
{
	var payload = [];
	var canId = getParamValue("canId", 0);

	if (canId > 0)
	{
		payload.push(COMM_FORWARD_CAN);
		payload.push(canId & 0xFF);
	}

	payload.push(command & 0xFF);
	return payload;
}

function sendPayload(payload)
{
	if(!local.parameters.isConnected.get()) return;
	var frame = [];
	var payloadLength = payload.length;
	var crc = crc16(payload);
	var i;

	if (typeof local.sendBytes !== "function")
	{
		setStatus("local.sendBytes is not available on this module");
		return;
	}

	if (payloadLength <= 255)
	{
		frame.push(START_SHORT);
		frame.push(payloadLength & 0xFF);
	}
	else
	{
		frame.push(START_LONG);
		frame.push((payloadLength >> 8) & 0xFF);
		frame.push(payloadLength & 0xFF);
	}

	for (i = 0; i < payloadLength; i++)
	{
		frame.push(payload[i] & 0xFF);
	}

	frame.push((crc >> 8) & 0xFF);
	frame.push(crc & 0xFF);
	frame.push(END_BYTE);

	if (isRawLoggingEnabled())
	{
		script.log("TX: " + frame.join(" "));
	}

	local.sendBytes(frame);
}

function processRxBuffer()
{
	var payloadLength;
	var frameLength;
	var payloadStart;
	var payload;
	var tempBuffer;
	var rxCrc;
	var computedCrc;

	while (rxBuffer.length >= 5)
	{
		if (rxBuffer[0] !== START_SHORT && rxBuffer[0] !== START_LONG)
		{
			consumeRxBytes(1);
			continue;
		}

		if (rxBuffer[0] === START_SHORT)
		{
			if (rxBuffer.length < 2)
			{
				return;
			}

			payloadLength = rxBuffer[1] & 0xFF;
			frameLength = payloadLength + 5;
			payloadStart = 2;
		}
		else
		{
			if (rxBuffer.length < 3)
			{
				return;
			}

			payloadLength = ((rxBuffer[1] & 0xFF) << 8) | (rxBuffer[2] & 0xFF);
			frameLength = payloadLength + 6;
			payloadStart = 3;
		}

		if (rxBuffer.length < frameLength)
		{
			return;
		}

		if (rxBuffer[frameLength - 1] !== END_BYTE)
		{
			setStatus("UART resync: invalid end byte");
			consumeRxBytes(1);
			continue;
		}

		tempBuffer = cloneArray(rxBuffer);
		payload = tempBuffer.splice(payloadStart, payloadLength);
		rxCrc = ((rxBuffer[frameLength - 3] & 0xFF) << 8) | (rxBuffer[frameLength - 2] & 0xFF);
		computedCrc = crc16(payload);

		if (isDecodedLoggingEnabled())
		{
			script.log("Frame candidate len=" + frameLength + ", payloadLen=" + payloadLength + ", crcRx=" + rxCrc + ", crcCalc=" + computedCrc);
		}

		if (rxCrc !== computedCrc)
		{
			setStatus("CRC mismatch: got " + rxCrc + ", expected " + computedCrc);
			consumeRxBytes(1);
			continue;
		}

		handlePayload(payload);
		rxBuffer.splice(0, frameLength);
	}
}

function consumeRxBytes(count)
{
	if (count <= 0)
	{
		return;
	}
	rxBuffer.splice(0, count);
}

function cloneArray(source)
{
	var result;
	var i;

	result = [];

	for (i = 0; i < source.length; i++)
	{
		result[i] = source[i];
	}

	return result;
}

function handlePayload(payload)
{
	var packetOffset = 0;
	var forwardedCanId = -1;
	var packetId;
	var body;
	var telemetry;
	var tempPayload;

	if (payload.length === 0)
	{
		return;
	}

	if (payload[0] === COMM_FORWARD_CAN && payload.length >= 3)
	{
		forwardedCanId = payload[1] & 0xFF;
		packetOffset = 2;
	}

	packetId = payload[packetOffset] & 0xFF;
	tempPayload = cloneArray(payload);
	body = tempPayload.splice(packetOffset + 1, tempPayload.length - (packetOffset + 1));

	if (isDecodedLoggingEnabled())
	{
		script.log("Decoded packet id=" + packetId + " (" + commandName(packetId) + "), payloadLen=" + payload.length + ", bodyLen=" + body.length + formatForwardedSuffix(forwardedCanId));
	}

	if (packetId === COMM_FW_VERSION)
	{
		handleFwVersion(body, forwardedCanId);
		return;
	}

	if (packetId === COMM_GET_VALUES)
	{
		telemetry = parseGetValues(body);
		if (telemetry !== null)
		{
			applyTelemetry(telemetry, forwardedCanId);
		}
		return;
	}

	setStatus("Unhandled packet: " + packetId + " (" + commandName(packetId) + ")");
}

function handleFwVersion(body, forwardedCanId)
{
	if (body.length < 2)
	{
		setStatus("FW version response too short");
		return;
	}

	setModuleValue("fwMajor", body[0] & 0xFF);
	setModuleValue("fwMinor", body[1] & 0xFF);
	if (isDecodedLoggingEnabled())
	{
		script.log("FW decoded: major=" + (body[0] & 0xFF) + ", minor=" + (body[1] & 0xFF) + formatForwardedSuffix(forwardedCanId));
	}
	setStatus("FW " + body[0] + "." + body[1] + formatForwardedSuffix(forwardedCanId));
}

function parseGetValues(body)
{
	var index = { value: 0 };
	var values = {};

	if (body.length < 55)
	{
		setStatus("COMM_GET_VALUES response too short: " + body.length + " bytes");
		return null;
	}

	values.tempMosfet = getFloat16(body, 10.0, index);
	values.tempMotor = getFloat16(body, 10.0, index);
	values.avgMotorCurrent = getFloat32(body, 100.0, index);
	values.avgInputCurrent = getFloat32(body, 100.0, index);
	index.value += 4;
	index.value += 4;
	values.dutyCycleNow = getFloat16(body, 1000.0, index);
	values.rpm = getFloat32(body, 1.0, index);
	values.inpVoltage = getFloat16(body, 10.0, index);
	values.ampHours = getFloat32(body, 10000.0, index);
	values.ampHoursCharged = getFloat32(body, 10000.0, index);
	values.wattHours = getFloat32(body, 10000.0, index);
	values.wattHoursCharged = getFloat32(body, 10000.0, index);
	values.tachometer = getInt32(body, index);
	values.tachometerAbs = getInt32(body, index);
	values.error = body[index.value++] & 0xFF;
	values.pidPos = getFloat32(body, 1000000.0, index);
	values.id = body[index.value++] & 0xFF;
	values.power = values.inpVoltage * values.avgInputCurrent;

	if (isDecodedLoggingEnabled())
	{
		script.log("Telemetry parsed: rpm=" + values.rpm + ", vin=" + values.inpVoltage + ", duty=" + values.dutyCycleNow + ", motorCurrent=" + values.avgMotorCurrent + ", inputCurrent=" + values.avgInputCurrent + ", mosfetTemp=" + values.tempMosfet + ", motorTemp=" + values.tempMotor + ", tach=" + values.tachometer + ", fault=" + values.error + ", id=" + values.id);
	}

	return values;
}

function applyTelemetry(values, forwardedCanId)
{
	if (isDecodedLoggingEnabled())
	{
		script.log("Applying telemetry to module values" + formatForwardedSuffix(forwardedCanId));
	}

	setModuleValue("rpm", Math.round(values.rpm));
	setModuleValue("inputVoltage", values.inpVoltage);
	setModuleValue("dutyCycle", values.dutyCycleNow);
	setModuleValue("motorCurrent", values.avgMotorCurrent);
	setModuleValue("inputCurrent", values.avgInputCurrent);
	setModuleValue("mosfetTemp", values.tempMosfet);
	setModuleValue("motorTemp", values.tempMotor);
	setModuleValue("ampHours", values.ampHours);
	setModuleValue("ampHoursCharged", values.ampHoursCharged);
	setModuleValue("wattHours", values.wattHours);
	setModuleValue("wattHoursCharged", values.wattHoursCharged);
	setModuleValue("tachometer", values.tachometer);
	setModuleValue("tachometerAbs", values.tachometerAbs);
	setModuleValue("pidPos", values.pidPos);
	setModuleValue("controllerId", values.id);
	setModuleValue("faultCode", values.error);
	setModuleValue("fault", faultCodeName(values.error));
	setModuleValue("power", values.power);

	if (isDecodedLoggingEnabled())
	{
		script.log("Applied parameters: Value RPM=" + Math.round(values.rpm) + ", Value Input Voltage=" + values.inpVoltage + ", Value Duty=" + values.dutyCycleNow + ", Value Motor Current=" + values.avgMotorCurrent + ", Value Input Current=" + values.avgInputCurrent + ", Fault=" + faultCodeName(values.error));
	}

	setStatus("Telemetry updated" + formatForwardedSuffix(forwardedCanId));
}

function normalizeIncomingBytes(data)
{
	var bytes = [];
	var i;

	if (data === null || data === undefined)
	{
		return bytes;
	}

	if (typeof data === "string")
	{
		for (i = 0; i < data.length; i++)
		{
			bytes.push(data.charCodeAt(i) & 0xFF);
		}
		return bytes;
	}

	if (typeof data.length === "number")
	{
		for (i = 0; i < data.length; i++)
		{
			bytes.push(data[i] & 0xFF);
		}
	}

	return bytes;
}

function appendInt32(buffer, value)
{
	var normalized = value | 0;
	buffer.push((normalized >> 24) & 0xFF);
	buffer.push((normalized >> 16) & 0xFF);
	buffer.push((normalized >> 8) & 0xFF);
	buffer.push(normalized & 0xFF);
}

function getInt16(buffer, index)
{
	var value = ((buffer[index.value] & 0xFF) << 8) | (buffer[index.value + 1] & 0xFF);
	index.value += 2;

	if (value & 0x8000)
	{
		value -= 0x10000;
	}

	return value;
}

function getInt32(buffer, index)
{
	var value = (buffer[index.value] << 24) |
		((buffer[index.value + 1] & 0xFF) << 16) |
		((buffer[index.value + 2] & 0xFF) << 8) |
		(buffer[index.value + 3] & 0xFF);

	index.value += 4;
	return value;
}

function getFloat16(buffer, scale, index)
{
	return getInt16(buffer, index) / scale;
}

function getFloat32(buffer, scale, index)
{
	return getInt32(buffer, index) / scale;
}

function crc16(buffer)
{
	var crc = 0;
	var i;
	var bit;

	for (i = 0; i < buffer.length; i++)
	{
		crc = crc ^ ((buffer[i] & 0xFF) << 8);

		for (bit = 0; bit < 8; bit++)
		{
			if ((crc & 0x8000) !== 0)
			{
				crc = ((crc << 1) ^ 0x1021) & 0xFFFF;
			}
			else
			{
				crc = (crc << 1) & 0xFFFF;
			}
		}
	}

	return crc & 0xFFFF;
}

function getModuleParameter(name)
{
	var target;

	if (!local || !local.parameters)
	{
		return null;
	}

	target = local.parameters[name];
	if (target)
	{
		return target;
	}

	return null;
}

function getModuleValue(name)

{
	var target;

	if (!local || !local.values)
	{
		return null;
	}

	target = local.values[name];
	if (target)
	{
		return target;
	}

	return null;
}

function getParamValue(name, fallback)

{
	var param = getModuleParameter(name);

	if (param && typeof param.get == "function")
	{
		return param.get();
	}

	return fallback;
}

function setModuleValue(name, value)

{
	var target = getModuleValue(name);

	if (target && typeof target.set == "function")
	{
		target.set(value);
		return true;
	}

	if (isDecodedLoggingEnabled())
	{
		script.log("No matching module value for " + name);
	}

	return false;
}

function isModuleParam(param, name)

{
	var target = getModuleParameter(name);

	if (!param || !target)
	{
		return false;
	}

	if (typeof param.is == "function")
	{
		return param.is(target);
	}

	return param === target;
}

function handleModuleTrigger(param)

{
	if (isModuleParam(param, "requestFw"))
	{
		sendSimpleCommand(COMM_FW_VERSION);
		return;
	}

	if (isModuleParam(param, "requestValues"))
	{
		sendSimpleCommand(COMM_GET_VALUES);
		return;
	}

	if (isModuleParam(param, "sendAlive"))
	{
		sendSimpleCommand(COMM_ALIVE);
		return;
	}

	if (isModuleParam(param, "applyControl"))
	{
		sendSelectedControl();
	}
}

function normalizeControlMode(mode)

{
	var value = ("" + mode).toLowerCase();
	while (value.indexOf("_") >= 0)
	{
		value = value.replace("_", " ");
	}

	while (value.indexOf("-") >= 0)
	{
		value = value.replace("-", " ");
	}

	while (value.indexOf("  ") >= 0)
	{
		value = value.replace("  ", " ");
	}

	while (value.length > 0 && value.charAt(0) === " ")
	{
		value = value.substring(1);
	}

	while (value.length > 0 && value.charAt(value.length - 1) === " ")
	{
		value = value.substring(0, value.length - 1);
	}

	return value;
}

function isRawLoggingEnabled()

{
	return getParamValue("logRaw", false);
}

function isDecodedLoggingEnabled()

{
	return getParamValue("logDecoded", true);
}

function setStatus(message)
{
	setModuleValue("status", message);
	script.log(message);
}

function formatForwardedSuffix(forwardedCanId)
{
	if (forwardedCanId < 0)
	{
		return "";
	}

	return " from CAN " + forwardedCanId;
}

function commandName(command)
{
	if (command == COMM_FW_VERSION)
	{
		return "COMM_FW_VERSION";
	}

	if (command == COMM_GET_VALUES)
	{
		return "COMM_GET_VALUES";
	}

	if (command == COMM_SET_DUTY)
	{
		return "COMM_SET_DUTY";
	}

	if (command == COMM_SET_CURRENT)
	{
		return "COMM_SET_CURRENT";
	}

	if (command == COMM_SET_CURRENT_BRAKE)
	{
		return "COMM_SET_CURRENT_BRAKE";
	}

	if (command == COMM_SET_RPM)
	{
		return "COMM_SET_RPM";
	}

	if (command == COMM_ALIVE)
	{
		return "COMM_ALIVE";
	}

	if (command == COMM_FORWARD_CAN)
	{
		return "COMM_FORWARD_CAN";
	}

	return "COMMAND_" + command;
}

function faultCodeName(code)
{
	if (code == 0)
	{
		return "NONE";
	}

	if (code == 1)
	{
		return "OVER_VOLTAGE";
	}

	if (code == 2)
	{
		return "UNDER_VOLTAGE";
	}

	if (code == 3)
	{
		return "DRV";
	}

	if (code == 4)
	{
		return "ABS_OVER_CURRENT";
	}

	if (code == 5)
	{
		return "OVER_TEMP_FET";
	}

	if (code == 6)
	{
		return "OVER_TEMP_MOTOR";
	}

	if (code == 7)
	{
		return "GATE_DRIVER_OVER_VOLTAGE";
	}

	if (code == 8)
	{
		return "GATE_DRIVER_UNDER_VOLTAGE";
	}

	if (code == 9)
	{
		return "MCU_UNDER_VOLTAGE";
	}

	if (code == 10)
	{
		return "BOOTING_FROM_WATCHDOG_RESET";
	}

	if (code == 11)
	{
		return "ENCODER_SPI";
	}

	if (code == 12)
	{
		return "ENCODER_SINCOS_BELOW_MIN_AMPLITUDE";
	}

	if (code == 13)
	{
		return "ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE";
	}

	if (code == 14)
	{
		return "FLASH_CORRUPTION";
	}

	if (code == 15)
	{
		return "HIGH_OFFSET_CURRENT_SENSOR_1";
	}

	if (code == 16)
	{
		return "HIGH_OFFSET_CURRENT_SENSOR_2";
	}

	if (code == 17)
	{
		return "HIGH_OFFSET_CURRENT_SENSOR_3";
	}

	if (code == 18)
	{
		return "UNBALANCED_CURRENTS";
	}

	if (code == 19)
	{
		return "BRK";
	}

	if (code == 20)
	{
		return "RESOLVER_LOT";
	}

	if (code == 21)
	{
		return "RESOLVER_DOS";
	}

	if (code == 22)
	{
		return "RESOLVER_LOS";
	}

	if (code == 23)
	{
		return "FLASH_CORRUPTION_APP_CFG";
	}

	if (code == 24)
	{
		return "FLASH_CORRUPTION_MC_CFG";
	}

	if (code == 25)
	{
		return "ENCODER_NO_MAGNET";
	}

	if (code == 26)
	{
		return "ENCODER_MAGNET_TOO_STRONG";
	}

	if (code == 27)
	{
		return "PHASE_FILTER";
	}

	if (code == 28)
	{
		return "ENCODER_FAULT";
	}

	if (code == 29)
	{
		return "LV_OUTPUT_FAULT";
	}

	return "FAULT_" + code;
}
