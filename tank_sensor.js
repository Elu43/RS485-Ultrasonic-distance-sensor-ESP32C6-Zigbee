const {numeric} = require('zigbee-herdsman-converters/lib/modernExtend');

const definition = {
    zigbeeModel: ['esp32c6'],
    model: 'esp32c6_urm14',
    vendor: 'DIY',
    description: 'URM14 distance sensor on ESP32-C6. Raw value is published in 0.1 mm.',
    extend: [
        numeric({
            name: 'distance',
            cluster: 'msIlluminanceMeasurement',
            attribute: 'measuredValue',
            description: 'Distance',
            unit: 'mm',
            access: 'STATE_GET',
            precision: 1,
            scale: 10,
            reporting: {min: 60, max: 3600, change: 10},
        }),
    ],
    meta: {},
};

module.exports = definition;
