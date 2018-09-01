const electron = require("electron");
const url = require('url');
const path = require('path');

const {app, BrowserWindow, Menu, ipcMain} = electron;

// Build menu from template
const mainMenu = Menu.buildFromTemplate(mainMenuTemplate);
// Insert Menu
Menu.setApplicationMenu(mainMenu);

const mainMenuTemplate = [
  {
    label: 'file',
    submenu: [
      {
        label: 'Quit',
        accelerator: process.platform == 'darwin' ? 'Command+Q' :
        'Ctrl+Q',
        click(){
          app.quit();
        }
      },
    ]
  }
];
