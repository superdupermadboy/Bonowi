const SerialPort = require('serialport');
const { MessagePortMain } = require('electron');


let port;

const sendToSerialPort = (data) => {
    console.log('sending this to serial port', data);
    
    if (port && !port.isOpen()) {
        port = new SerialPort('COM4', {
            baudRate: 9600,
            autoOpen: false,
        });    
    }


    port.open((err) => {
        console.log('Error happened while opening port: ', err);
    });

    port.write(data);

    port.on('open', () => {
        port.close();
    });
}

const getAllSerialPorts = () => {
    SerialPort.list().then(data => {
        console.log(data);
    })
}

module.exports = {
    sendToSerialPort: sendToSerialPort,
    getAllSerialPorts: getAllSerialPorts,
}
