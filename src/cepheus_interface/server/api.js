// import packages
const express = require('express');
const rosnodejs = require('rosnodejs');
const shell = require('shelljs');
const { spawn } = require('child_process');


module.exports = function(io) {

	const router = express.Router();
	
	router.post('/start', function(req, res, next) {
		console.log('# start #')
		var child = spawn('sh', ['prescript.sh', './start_robot.sh'], {cwd: './scripts'})
		child.stdout.on('data', (data) => {
			io.emit('status', 'running');
			// io.emit('log', data);
		});

		child.stderr.on('data', (data) => {
			io.emit('status', 'error');
			// io.emit('log', data);
		});

		child.on('close', (code) => {
			// io.emit('status', 'closed');
			// io.emit('log', code);
		});

	})

	router.post('/stop', function(req, res, next) {
		console.log('# stop #')
		var child = spawn('sh', ['prescript.sh', './stop_robot.sh'], {cwd: './scripts'})
		// child.stdout.on('data', (data) => {
		// 	io.emit('status', 'stopped');
		// 	// io.emit('log', data);
		// });

		// child.stderr.on('data', (data) => {
		// 	io.emit('status', 'error');
		// 	// io.emit('log', data);
		// });

		// child.on('close', (code) => {
		// 	// io.emit('status', 'closed');
		// 	// io.emit('log', code);
		// });
	})

	router.post('/startSimulation', function(req, res, next) {
		var controller = req.body.controller ? true : false;
		shell.cd('../../..');
		shell.exec(`./start_local_simulation.sh ${controller}`, function(code, stdout, stderr) {
			shell.cd('./src/cepheus_interface/server');
			if (stderr) {
				res.send({ status: 'error' });
				io.emit('status', 'error');
				io.emit('log', stderr);
			} else {
				res.json({ status: 'running' });
				io.emit('status', 'running');
				io.emit('log', stdout);
			}
		});
	});

	router.post('/stopSimulation', function(req, res, next) {	
		shell.exec('killall -9 gzserver gzclient; rosnode kill -a; killall rosmaster', function(code, stdout, stderr) {
			if (stderr) {
				res.send({ status: 'error' });
				io.emit('status', 'error');
				io.emit('log', stderr);
			} else {
				res.json({ status: 'stopped' });
				io.emit('status', 'stopped');
				io.emit('log', stdout);
			}
		});
	});

	return router;
};
