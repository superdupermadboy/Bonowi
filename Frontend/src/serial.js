const SerialPort = require('serialport');

let port;

const sendToSerialPort = (data) => {
    console.log('sending this to serial port', data);
    
    let portWritten = port.write(data, (error) => {
        console.log('Written to port with error', error)
    });

    console.log('written status', portWritten);
}

const openPort = (portPath) => {
    console.log('opening port', portPath)

    port = SerialPort(portPath, {
        baudRate: 9600,
        autoOpen: false,
    });


    port.open(error => {
        console.log('Port openend with error', error);
    });
}

const closePort = () => {
    port.close((error) => {
        console.log('Port closed with error', error)
    });
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
