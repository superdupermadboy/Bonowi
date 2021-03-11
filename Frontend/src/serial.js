const SerialPort = require('serialport');
const Readline = require('@serialport/parser-readline')
const { createLogger, format, transports } = require('winston');
const { json, timestamp, combine } = format;
const { EventEmitter } = require('events');

const emitter = new EventEmitter();

const logger = createLogger({
  level: 'info',
  format: combine(
      timestamp(),
      json(),
  ),
  transports: [
    new transports.File({ filename: `${require('os').homedir()}/.bonowi-lamp-updater.log` }),
  ],
});

let port;

const receiveData = (data) => {
    response  = {
        error: false,
        message: data,
    }

    if (data !== 'Success!') {
        response.error = true;
        logger.error(`µController didn't understand the command and returned :${data}`);
    } else {
        logger.info('µController successfully changed mode');
    }

    emitter.emit('controller-response', response);
}

const sendToSerialPort = (data) => {

    // TODO port.write returns flase if port.drain should be called. Is this relevant for our use case?
    logger.info(`Sending '${data}' to comport`);

    return new Promise(resolve => port.write(data, error => {
        if (error) {
            logger.error(`Error while sending ${data}: ${error}`);
            resolve(false);
            return;
        } 

        resolve(true);
    }));
}

const openPort = (portPath) => {
    logger.info(`Opening comport '${portPath}'`);


    port = SerialPort(portPath, {
        baudRate: 9600,
        autoOpen: false,
    });


    return new Promise(resolve => port.open(error => {
        if (error) {
            logger.error(`Error while opening comport: ${error}`);
            resolve(false);
            return;
        } 

        const parser = port.pipe(new Readline({ delimiter: '#' }));

        parser.on('data', receiveData);

        resolve(true);
    }));
}

const closePort = () => {
    logger.info('Closing port');

    return new Promise(resolve => port.close((error) => {
        if (error) {
            logger.error(`Error while closing port: ${error}`);
            resolve(false);
            return;
        } 

        resolve(true);
    }))
}

const getAllSerialPorts = async () => {

    const comPorts = await SerialPort.list();
    
    return comPorts;
}

module.exports = {
    openPort: openPort,
    closePort: closePort,
    sendToSerialPort: sendToSerialPort,
    getAllSerialPorts: getAllSerialPorts,
    emitter: emitter,
}
