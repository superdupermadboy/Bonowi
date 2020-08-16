const SerialPort = require('serialport');
const { MessagePortMain } = require('electron');


let port;

const sendToSerialPort = (data) => {
    console.log('sending this to serial port', data);
    
    port = new SerialPort(data.comport, {
        baudRate: 9600,
        autoOpen: false,
    });     

    port.open((err) => {
        console.log('Error happened while opening port: ', err);
    });

    port.on('open', () => {
        console.log('sending data');
        port.write(data.mode);
        port.close();
    });
}

const getAllSerialPorts = async () => {

    const comPorts = await SerialPort.list();
    
    return comPorts;
}

module.exports = {
    sendToSerialPort: sendToSerialPort,
    getAllSerialPorts: getAllSerialPorts,
}
