const SerialPort = require('serialport');
const { ipcRenderer, ipcMain } = require('electron');

let port;

const sendToSerialPort = (data) => {
    console.log('sending this to serial port', data);
    
    let portWritten = port.write(data, (error) => {
        console.log('Written to port with error', error)
    });

    console.log('written status', portWritten);
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
