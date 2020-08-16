// All of the Node.js APIs are available in the preload process.
// It has the same sandbox as a Chrome extension.
const { ipcRenderer } = require('electron');
const { getAllSerialPorts } = require('../src/serial');

window.addEventListener('DOMContentLoaded', () => {
    let comportList = document.getElementById('comport');

    getAllSerialPorts().then(allComports => {

        allComports.forEach(comport => {

            const listEntry = document.createElement('option');
            listEntry.appendChild(document.createTextNode(comport.path));
            listEntry.value = comport.path;

            comportList.appendChild(listEntry);
        })
    
    })
    const saveButton = document.getElementById('button-save');

    saveButton.addEventListener('click', () => {
        const mode = document.getElementById('mode');
        comportList = document.getElementById('comport');

        ipcRenderer.send('update-lamp', {
            comport: comportList.options[comportList.selectedIndex].value,
            mode: mode.options[mode.selectedIndex].value,
        });
    })
  })
  