const electron = require("electron");
const url = require('url');
const path = require('path');

const {app, BrowserWindow, Menu, ipcMain} = electron;

// SET ENV
// process.env.NODE_ENV = 'production';

let mainWindow;
let addWindow;

// Listen for app to be ready
app.on('ready', function(){

  openingWindow = new BrowserWindow({});

  openingWindow.loadURL(url.format({
    pathname: path.join(__dirname, 'openingWindow.html'),
    protocol: 'file:',
    slashes: true
  }));

  // Build menu from template
  const mainMenu = Menu.buildFromTemplate(mainMenuTemplate);
  // Insert Menu
  Menu.setApplicationMenu(mainMenu);
});

ipcMain.on('main', (event, arg) => {
  //create new window
  if (arg == "launch") {

    mainWindow = new BrowserWindow({});
    // Load html into mainWindow
    mainWindow.loadURL(url.format({
      pathname: path.join(__dirname, 'mainWindow.html'),
      protocol:'file:',
      slashes: true
    }));
    // Quit app when closed
    mainWindow.on('closed', function(){
      app.quit();
    });
    console.log(arg);
  }

  if (arg == "engine_test") {
    app.quit();
    //console.log("engine testing ain't available yet cuz");

    //engineTestWindow = new BrowserWindow({});

    //engineTestWindow.loadURL(url.format({
    //  pathname: path.join(__dirname, 'getfucked.html'),
    //  protocol:'file:',
    //  slashes: true
    //}));
  }

  if (arg == "saved_check") {
    console.log(arg);
  }

  });



// Handle create add window
function createAddWindow(){
  //create new window
  addWindow = new BrowserWindow({
    width:600,
    height:500,
    title:'Configure'
  });
  // Load html into mainWindow
  addWindow.loadURL(url.format({
    pathname: path.join(__dirname, 'addWindow.html'),
    protocol:'file:',
    slashes: true
  }));
  // Garbage collection Handle
  addWindow.on('close', function(){
    addWindow = null;
  });
}

// Create menu template
const mainMenuTemplate = [
  {
    label:'File',
    submenu:[
      {
        label: 'Setup',
        accelerator: process.platform == 'darwin' ? 'Command+T' :
        'Ctrl+T',
        click(){
          createAddWindow();
        }
      },
      {
        label: 'Clear Items',
        click(){
          mainWindow.webContents.send('masstarget:clear');
        }
      },
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

// plots



// If mac, add empty object to Menu
if(process.platform == 'darwin'){
  mainMenuTemplate.unshift({});
}

// Add developer tools item if not in production
if(process.env.NODE_ENV !== 'production'){
  mainMenuTemplate.push({
    label: 'Developer Tools',
    submenu: [
      {
        label: 'Toggle DevTools',
        accelerator: process.platform == 'darwin' ? 'Command+I' :
        'Ctrl+I',
        click(item, focusedWindow){
          focusedWindow.toggleDevTools();
        }
      },
      {
        role: 'reload'
      }
    ]
  });
}
