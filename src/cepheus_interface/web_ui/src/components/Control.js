import React, { useState, useEffect } from 'react';
import { withRouter } from 'react-router';
// import { Link } from 'react-router-dom';
import io from 'socket.io-client';
import ROSLIB from 'roslib';

var ros;

function Control() {

	const [response, setResponse] = useState("");
	const [started, setStarted] = useState("");
	const [loading, setLoading] = useState("");
	const [socketUp, setSocketUp] = useState("");
	const [rw_velocity, setRWVelocity] = useState("");


	useEffect(() => {

		const socket = io('http://localhost:9000');

		socket.on('status', data => {
			if (data === 'running') {
				setLoading(false);
				setStarted(true);
			}
			else if (data === 'stopped') {
				setLoading(false);
				setStarted(false);
			}

		});

		socket.on('log', data => {
			setResponse(data);
		});

	}, []);


	const startRobot = (e) => {

		e.preventDefault();
		setLoading(true);
		setSocketUp(false);

		let requestOptions = {
			method: 'POST'
		};

		fetch(`http://localhost:9000/start`, requestOptions)
		.then(status => {
			setTimeout(function() {
				
				ros = new ROSLIB.Ros({
					url : 'ws://localhost:9090'
				});

				ros.on('connection', function() {
					setSocketUp(true);
					console.log('Connected to websocket server.');
				});

				ros.on('error', function(error) {
					setSocketUp(false);
					console.log('Error connecting to websocket server: ', error);
				});

				ros.on('close', function() {
					setSocketUp(false);
					console.log('Connection to websocket server closed.');
				});


			}, 7000);
		})

	};

	const stopRobot = (e) => {

		e.preventDefault();
		// setLoading(true);

		let requestOptions = {
			method: 'POST'
		};

		fetch(`http://localhost:9000/stop`, requestOptions);
	}

	const handleRWVelocitySubmit = (e) => {

		console.log(typeof ros);
		e.preventDefault();
		let rw_vel_topic = new ROSLIB.Topic({
			ros : ros,
			name : '/cepheus/reaction_wheel_velocity_controller/command',
			messageType : 'std_msgs/Float64'
		});
		let velocity = new ROSLIB.Message({
			data : parseFloat(rw_velocity)
		});
		rw_vel_topic.publish(velocity);
	}

	return (
		<div>
			<div style={{ textAlign: 'center' }}>
				<p>Cepheus Simulation</p>
				{!started &&
					<button onClick={startRobot} className="btn btn-primary shadow mb-0">
						{!loading && <span>Start</span>}
						{loading && <span>Starting...</span>}
					</button>
				}
				{started &&
					<button onClick={stopRobot} className="btn btn-danger shadow mb-0">
						{!loading && <span>Stop</span>}
						{loading && <span>Stopping...</span>}
					</button>
				}
			</div>
			{(started && socketUp) &&
				<form  className="form-inline" onSubmit={handleRWVelocitySubmit}>
					<div className="form-group mx-sm-3 mb-2">
						<label htmlFor="rw_vel"></label>
						<input
							className="form-control"
							name="velocity"
							id="rw_vel"
							type="number"
							placeholder="Set RW Velocity(rad/s)"
							value={rw_velocity}
							onChange={e => setRWVelocity(e.target.value)} />
					</div>
					<button type="submit" className="btn btn-primary  mb-2">Send Velocity</button>
				</form>
			}
			<br/>
			<div className="content-section">
				Logs:<br/>
				<small className="float-left" style={{whiteSpace: 'pre-line'}}>
					{response}
				</small>
			</div>
		</div>
	);
}

export default withRouter(Control);
