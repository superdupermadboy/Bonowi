// All of the Node.js APIs are available in the preload process.
// It has the same sandbox as a Chrome extension.
const { ipcRenderer } = require('electron');
const { getAllSerialPorts } = require('../src/serial');

window.addEventListener('DOMContentLoaded', () => {
    let comportList = document.getElementById('comport');

    getAllSerialPorts().then(allComports => {

        allComports.forEach(comport => {

            const listEntry = document.createElement('option');
            listEntry.appendChild(document.createTextNode(
                `${comport.path} - ${comport.manufacturer}`
            ));
            listEntry.value = comport.path;

            comportList.appendChild(listEntry);
        })
    
    })

    const openPortButton = document.getElementById('open-port');

    openPortButton.addEventListener('click', () => {
        comportList = document.getElementById('comport');
        openPortButton.disabled = true;

        ipcRenderer.send('open-port', comportList.options[comportList.selectedIndex].value);
    })

    const closePortButton = document.getElementById('close-port');

    closePortButton.addEventListener('click', () => {
        ipcRenderer.send('close-port');
    })

    const saveButton = document.getElementById('button-save');

    saveButton.addEventListener('click', () => {
        const mode = document.getElementById('mode');

        ipcRenderer.send('update-lamp', mode.options[mode.selectedIndex].value);
    })

    ipcRenderer.on('success-opening-port', () => {
        closePortButton.disabled = false;
    });

    ipcRenderer.on('error-opening-port', () => {
        openPortButton.disabled = false;
    });

    ipcRenderer.on('success-closing-port', () => {
        closePortButton.disabled = true;
        openPortButton.disabled = false;
    });

    ipcRenderer.on('error-opening-port', () => {
        openPortButton.disabled = false;
    });
  })
  