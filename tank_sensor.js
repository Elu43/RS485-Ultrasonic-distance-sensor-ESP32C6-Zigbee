const {numeric} = require('zigbee-herdsman-converters/lib/modernExtend');

const definition = {
    zigbeeModel: ['esp32c6'],  // modelId annoncé par le firmware (CONFIG_IDF_TARGET)
    model: 'esp32c6_a02_rs485',
    vendor: 'DIY',
    description: 'A02 RS485 Modbus distance sensor on ESP32-C6 (XIAO). Distance is published in mm via msIlluminanceMeasurement/measuredValue.',
    extend: [
        numeric({
            name: 'distance',
            cluster: 'msIlluminanceMeasurement',
            attribute: 'measuredValue',
            description: 'Distance',
            unit: 'mm',
            access: 'STATE_GET',
            precision: 0,
            scale: 1,
            reporting: {min: 5, max: 0, change: 10}, // min=5s; change=10mm (modifiable depuis Z2M)
        }),
    ],
    meta: {},
};

module.exports = definition;
