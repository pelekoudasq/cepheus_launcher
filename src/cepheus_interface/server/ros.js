// import packages
const argv = require('minimist')(process.argv.slice(2));
const bodyParser = require('body-parser');
const express = require('express');
const fs = require("fs");
const http = require('http');
const shell = require('shelljs');
const socketio = require('socket.io');

require('dotenv').config();

const app = express();
const server = http.createServer(app);
const io = socketio(server);

if (argv.robot_address) {
	console.log(`Pinging robot computer at: ${argv.robot_address}`);
	setInterval(function(){
		shell.exec(`ping -c 1 ${argv.robot_address}`,
			{ silent: true, async: true },
			function(code, stdout, stderr) {
				io.emit('ping', stdout);
		})
	}, 1000);
}

const api = require('./api')(io);

const port = 9000;

app.use(function(req, res, next) {
	res.header("Access-Control-Allow-Origin", process.env.FRONT_URI);
	res.header("Access-Control-Allow-Headers", "Origin, X-Requested-With, Content-Type, Accept, Access-Control-Allow-Headers, Access-Control-Request-Method, Access-Control-Request-Headers, Authorization");
	res.header('Access-Control-Allow-Methods', 'GET, PUT, POST, DELETE, PATCH, OPTIONS');
	next();
});

//Body Parsers middle ware
app.use(bodyParser.urlencoded({limit: '50mb', extended: true}));
app.use(bodyParser.json({limit: '50mb', extended: true}));

app.use('/', api);

server.listen(port, function(){
	console.log(`Server started on port ${port}`);
});
