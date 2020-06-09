// import packages
const express = require('express');
const rosnodejs = require('rosnodejs');
const shell = require('shelljs');


module.exports = function(io) {

	const router = express.Router();
	
	router.post('/start', function(req, res, next) {
		shell.cd('../../..');
		shell.exec('./start.sh', function(code, stdout, stderr) {
			shell.cd('./src/cepheus_interface/server');
			if (stderr) {
				// res.send(stderr);
				io.emit('status', 'error');
				io.emit('log', stderr);
			}
			// res.json({status : stdout});
			io.emit('status', 'running');
			io.emit('log', stdout);

			// setTimeout(function () {
			// 	const nh = rosnodejs.nh;
			// 	const sub = nh.subscribe('/cepheus/joint_states', 'sensor_msgs/JointState', (msg) => {
			// 		console.log('Got msg on c: %j', msg);
			// 	}, {queueSize: 1, throttleMs: 1000});
			// }, 10000);

		});
	});

	router.post('/stop', function(req, res, next) {	
		shell.exec('rosnode kill -a', function(code, stdout, stderr) {
			if (stderr) {
				// res.send(stderr);
				io.emit('status', 'error');
				io.emit('log', stderr);
			}
			// res.json({status : stdout});
			io.emit('status', 'stopped');
			io.emit('log', stdout);
		});
	});

	return router;
};
