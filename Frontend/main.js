// Modules to control application life and create native browser window
const {app, BrowserWindow, ipcMain} = require('electron')
const path = require('path')
const { sendToSerialPort, closePort, openPort } = require('./src/serial')

app.allowRendererProcessReuse = false;

function createWindow () {
  // Create the browser window.
  const mainWindow = new BrowserWindow({
    width: 800,
    height: 600,
    webPreferences: {
      preload: path.join(__dirname, 'components/preload.js')
    },
  })

  mainWindow.setMenu(null);

  // and load the index.html of the app.
  mainWindow.loadFile('components/index.html')

  // Open the DevTools.
  // mainWindow.webContents.openDevTools()
}

// This method will be called when Electron has finished
// initialization and is ready to create browser windows.
// Some APIs can only be used after this event occurs.
app.whenReady().then(() => {
  createWindow()
  
  app.on('activate', function () {
    // On macOS it's common to re-create a window in the app when the
    // dock icon is clicked and there are no other windows open.
    if (BrowserWindow.getAllWindows().length === 0) createWindow()
  })
})

// Quit when all windows are closed, except on macOS. There, it's common
// for applications and their menu bar to stay active until the user quits
// explicitly with Cmd + Q.
app.on('window-all-closed', function () {
  if (process.platform !== 'darwin') app.quit()
})

// In this file you can include the rest of your app's specific main process
// code. You can also put them in separate files and require them here.

ipcMain.on('update-lamp', async (event, data) => {
  success = await sendToSerialPort(data);

  response = '';

  if (success) {
    response = 'success-updating-lamp';
  } else {
    response = 'error-updating-lamp';
  }

  event.reply(response, data);
});

ipcMain.on('open-port', async (event, data) => {
  success = await openPort(data);

  response = '';

  if (success) {
    response = 'success-opening-port';
  } else {
    response = 'error-opening-port';
  }

  event.reply(response);
});

ipcMain.on('close-port', async (event, data) => {
  success = await closePort();

  response = '';

  if (success) {
    response = 'success-closing-port';
  } else {
    response = 'error-closing-port';
  }

  event.reply(response);
});
