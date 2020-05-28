// import packages
const express = require('express');
const rosnodejs = require('rosnodejs');
const shell = require('shelljs');

// declare vars
const router = express.Router();

router.post('/start', function(req, res, next) {
	shell.cd('../../..');
	shell.exec('./start.sh', function(code, stdout, stderr) {
		shell.cd('./src/cepheus_interface/server');
		if (stderr)
			res.send(stderr);
		res.json({status : stdout});
	});
});

router.post('/stop', function(req, res, next) {	
	shell.exec('rosnode kill -a', function(code, stdout, stderr) {
		if (stderr)
			res.send(stderr);
		res.json({status : stdout});
	});
});


module.exports = router;
