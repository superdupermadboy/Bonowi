const SerialPort = require('serialport');

let port;

const sendToSerialPort = (data) => {

    // TODO port.write returns flase if port.drain should be called. Is this relevant for our use case?

    return new Promise(resolve => port.write(data, error => {
        if (error) {
            resolve(false);
            return;
        } 

        resolve(true);
    }));
}

const openPort = (portPath) => {
    port = SerialPort(portPath, {
        baudRate: 9600,
        autoOpen: false,
    });


    return new Promise(resolve => port.open(error => {
        if (error) {
            resolve(false);
            return;
        } 

        resolve(true);
    }));
}

const closePort = () => {

    return new Promise(resolve => port.close((error) => {
        if (error) {
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
}
