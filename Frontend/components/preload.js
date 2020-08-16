// All of the Node.js APIs are available in the preload process.
// It has the same sandbox as a Chrome extension.
const { ipcRenderer } = require('electron');

window.addEventListener('DOMContentLoaded', () => {
    const saveButton = document.getElementById('button-save');

    saveButton.addEventListener('click', () => {
        const inputValue = document.getElementById('input-value').value;

        ipcRenderer.send('update-lamp', inputValue);
    })

  })
  