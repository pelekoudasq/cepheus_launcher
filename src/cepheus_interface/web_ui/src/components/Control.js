import React, { useState, useEffect } from 'react';
import { withRouter } from 'react-router';
// import { Link } from 'react-router-dom';
import io from 'socket.io-client';
import ROSLIB from 'roslib';

import '../App.css';


var ros;

function Control() {

	const [response, setResponse] = useState("");
	const [started, setStarted] = useState("");
	const [loading, setLoading] = useState("");
	const [socketUp, setSocketUp] = useState("");
	const [rw_velocity, setRWVelocity] = useState("");
	const [elbow_velocity, setElbowVelocity] = useState("");
	const [shoulder_velocity, setShoulderVelocity] = useState("");
	const [positionX, setPositionX] = useState("");
	const [positionY, setPositionY] = useState("");


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


			}, 10000);
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


	const handleShoulderVelocitySubmit = (e) => {

		e.preventDefault();
		let shoulder_vel_topic = new ROSLIB.Topic({
			ros : ros,
			name : '/cepheus/left_shoulder_position_controller/command',
			messageType : 'std_msgs/Float64'
		});
		let velocity = new ROSLIB.Message({
			data : parseFloat(shoulder_velocity)
		});
		shoulder_vel_topic.publish(velocity);
	}


	const handleElbowVelocitySubmit = (e) => {

		e.preventDefault();
		let elbow_vel_topic = new ROSLIB.Topic({
			ros : ros,
			name : '/cepheus/left_elbow_position_controller/command',
			messageType : 'std_msgs/Float64'
		});
		let velocity = new ROSLIB.Message({
			data : parseFloat(elbow_velocity)
		});
		elbow_vel_topic.publish(velocity);
	}


	const handlePositionXSubmit = (e) => {

		e.preventDefault();
		let position1_vel_topic = new ROSLIB.Topic({
			ros : ros,
			name : '/cepheus/p1_position_controller/command',
			messageType : 'std_msgs/Float64'
		});
		let pos1 = new ROSLIB.Message({
			data : parseFloat(positionX)
		});
		position1_vel_topic.publish(pos1);
	}


	const handlePositionYSubmit = (e) => {

		e.preventDefault();
		let position2_vel_topic = new ROSLIB.Topic({
			ros : ros,
			name : '/cepheus/p2_position_controller/command',
			messageType : 'std_msgs/Float64'
		});
		let pos2 = new ROSLIB.Message({
			data : parseFloat(positionY)
		});
		position2_vel_topic.publish(pos2);
	}


	return (
		<div className="col-md-12">
			<div className="border-bottom pt-3 mb-2">
				<p className="h4 text-center">Main Control Panel</p>
			</div>
			<div className="row">
				<div className="col-md-8 p-1">
					<div className="content-section container shadow mb-6" style={{height: '600px'}}>
						<div className="border-bottom pt-1 mb-1">
							<p className="h6">Top Down View</p>
						</div>
						Time: {}
						<br/>
						<br/>
						<br/>
						<br/>
						Live feed
					</div>
					<div className="px-3 mx-3" style={{ textAlign: 'center' }}>
						{!started &&
							<button disabled={loading} onClick={startRobot} className="btn btn-primary shadow mb-0 w-100">
								{!loading && <span>Start Robot</span>}
								{loading && <span>Starting...</span>}
							</button>
						}
						{started &&
							<button disabled={loading} onClick={stopRobot} className="btn btn-danger shadow mb-0 w-100">
								{!loading && <span>Emergency Stop</span>}
								{loading && <span>Stopping...</span>}
							</button>
						}
					</div>
				</div>
				<div className="col-md-4 p-1">
					<div className="content-section container shadow" style={{height: '360px'}}>
						<div className="border-bottom pt-1 mb-1">
							<p className="h6">Logs</p>
						</div>
						<small className="float-left" style={{whiteSpace: 'pre-line'}}>
							{response}
						</small>
					</div>
					<div className="content-section container shadow mb-6">
						<div className="border-bottom pt-1 mb-3">
							<p className="h6">Manual Override</p>
						</div>
						<form className="form-inline" onSubmit={handlePositionXSubmit}>
							<div className="form-group mx-sm-3 mb-2">
								<label htmlFor="posX"></label>
								<input
									className="form-control"
									name="positionX"
									id="posX"
									type="number"
									placeholder="Position X (m)"
									value={positionX}
									onChange={e => setPositionX(e.target.value)} />
							</div>
							<button disabled={(!started || !socketUp)} type="submit" className="btn btn-primary  mb-2">Send Position X</button>
						</form>
						<form className="form-inline" onSubmit={handlePositionYSubmit}>
							<div className="form-group mx-sm-3 mb-2">
								<label htmlFor="posY"></label>
								<input
									className="form-control"
									name="positionY"
									id="posY"
									type="number"
									placeholder="Position Y (m)"
									value={positionY}
									onChange={e => setPositionY(e.target.value)} />
							</div>
							<button disabled={(!started || !socketUp)} type="submit" className="btn btn-primary  mb-2">Send Position Y</button>
						</form>
						<form className="form-inline" onSubmit={handleRWVelocitySubmit}>
							<div className="form-group mx-sm-3 mb-2">
								<label htmlFor="rw_vel"></label>
								<input
									className="form-control"
									name="rw_velocity"
									id="rw_vel"
									type="number"
									placeholder="Reaction Wheel Velocity (rad/s)"
									value={rw_velocity}
									onChange={e => setRWVelocity(e.target.value)} />
							</div>
							<button disabled={(!started || !socketUp)} type="submit" className="btn btn-primary  mb-2">Send RW Velocity</button>
						</form>
						<form className="form-inline" onSubmit={handleShoulderVelocitySubmit}>
							<div className="form-group mx-sm-3 mb-2">
								<label htmlFor="shoulder_vel"></label>
								<input
									className="form-control"
									name="shoulder_velocity"
									id="shoulder_vel"
									type="number"
									placeholder="Shoulder Velocity (rad/s)"
									value={shoulder_velocity}
									onChange={e => setShoulderVelocity(e.target.value)} />
							</div>
							<button disabled={(!started || !socketUp)} type="submit" className="btn btn-primary  mb-2">Send Shoulder Velocity</button>
						</form>
						<form className="form-inline" onSubmit={handleElbowVelocitySubmit}>
							<div className="form-group mx-sm-3 mb-2">
								<label htmlFor="elbow_vel"></label>
								<input
									className="form-control"
									name="elbow_velocity"
									id="elbow_vel"
									type="number"
									placeholder="Elbow Velocity (rad/s)"
									value={elbow_velocity}
									onChange={e => setElbowVelocity(e.target.value)} />
							</div>
							<button disabled={(!started || !socketUp)} type="submit" className="btn btn-primary  mb-2">Send Elbow Velocity</button>
						</form>
					</div>
				</div>
				
				<br/>
				
			</div>
		</div>
	);
}

export default withRouter(Control);
