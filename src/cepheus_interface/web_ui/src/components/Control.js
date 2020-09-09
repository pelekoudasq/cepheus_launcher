import React, { useState, useEffect } from 'react';
import { withRouter } from 'react-router';
// import { Link } from 'react-router-dom';
import io from 'socket.io-client';
import ROSLIB from 'roslib';
// import 'mjpegcanvas';
import '../App.css';


var ros;

function Control() {

	const [panel, setPanel] = useState(0);
	const [lock, setLock] = useState(false);


	const [response, setResponse] = useState("");
	const [started, setStarted] = useState("");
	const [loading, setLoading] = useState(false);
	const [override, setOverride] = useState(false);
	const [logs, setLogs] = useState(false);
	const [ignite, setIgnite] = useState(true);
	const [socketUp, setSocketUp] = useState(false);

	const [controller, setController] = useState(false);
	const [rw_velocity, setRWVelocity] = useState("");
	const [elbow_position, setElbowPosition] = useState("");
	const [shoulder_position, setShoulderPosition] = useState("");
	const [positionX, setPositionX] = useState("");
	const [positionY, setPositionY] = useState("");

	const [sec, setTimeSec] = useState(0);
	const [nsec, setTimeNsec] = useState(0);

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
		setLock(true);

		let requestOptions = {
			mode: 'cors',
			method: 'POST',
			headers: { 'Content-Type': 'application/json' },
			body: JSON.stringify({
				controller
			}),
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
					var time_listener = new ROSLIB.Topic({
						ros : ros,
						name : '/clock',
						messageType : 'rosgraph_msgs/Clock'
					});
					let interval = setInterval(() => {
						time_listener.subscribe(function(message) {
							setTimeSec(message.clock.secs);
							setTimeNsec(message.clock.nsecs);
							time_listener.unsubscribe();
						});
					}, 500);
					return () => {
						clearInterval(interval);
					};
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


	const handleControllerSelector = (e) => {
		setController(e.target.value);
	}


	const handlePanelSelector = (e) => {
		setPanel(e.target.value);
		setLock(true);
	}


	const stopRobot = (e) => {

		e.preventDefault();
		setLoading(true);
		setTimeSec(0);
		setTimeNsec(0);

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


	const handleShoulderPositionSubmit = (e) => {

		e.preventDefault();
		let shoulder_pos_topic = new ROSLIB.Topic({
			ros : ros,
			name : '/cepheus/left_shoulder_position_controller/command',
			messageType : 'std_msgs/Float64'
		});
		let pos = new ROSLIB.Message({
			data : parseFloat(shoulder_position)
		});
		shoulder_pos_topic.publish(pos);
	}


	const handleElbowPositionSubmit = (e) => {

		e.preventDefault();
		let elbow_pos_topic = new ROSLIB.Topic({
			ros : ros,
			name : '/cepheus/left_elbow_position_controller/command',
			messageType : 'std_msgs/Float64'
		});
		let pos = new ROSLIB.Message({
			data : parseFloat(elbow_position)
		});
		elbow_pos_topic.publish(pos);
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
				<form>


					<div className="form-row">
						<div className="ml-auto mr-auto col-auto">
							<p className="h4 text-center form-group">Main Control Panel</p>
						</div>
						<div className="col-auto">
							{lock &&
								<button className="btn btn-info" onClick={e => setLock(!lock)}>
									<i className="fa fa-unlock-alt" aria-hidden="true"></i>
								</button>
							}
						</div>
						<div className="col-auto">
							<select
								className="form-control mb-2" 
								id="panelSelector"
								value={panel}
								onChange={handlePanelSelector}
								disabled={lock}>
								<option value="1">Simulation</option>
								<option value="2">Cepheus Robot</option>
							</select>
						</div>
					</div>
				</form>
			</div>
			<div className="row">
				<div className="col-md-8 p-1">
					<div className="content-section container shadow mb-6" style={{height: '715px'}}>
						<div className="border-bottom pt-1 mb-1">
							<p className="h6">Top Down View</p>
						</div>
						Simulation Time: {sec} <small>secs</small>, {nsec} <small>nsecs</small>
						<br/>
						{(started && socketUp) &&
							<iframe
								className="w-100"
								style={{position: 'relative', height: '90%'}}
								src="http://localhost:8080/stream?topic=/rrbot/camera1/image_raw&width=1093&height=600"/>
							}
					</div>
					<div className="px-3 mx-3" style={{ textAlign: 'center' }}>
						{started &&
							<button
								disabled={loading}
								onClick={stopRobot}
								className="btn btn-danger shadow mb-0 w-100">
								{!loading &&
									<span>Emergency Stop</span>}
								{loading &&
									<span>Stopping...</span>}
							</button>
						}
					</div>
				</div>
				<div className="col-md-4 p-1">
					<div className="content-section container shadow mb-6">
						<div className="border-bottom pt-1 mb-3">
							<p className="h6">
								<input
									className="mr-2"
									name="override"
									type="checkbox"
									defaultChecked={ignite} 
									value={ignite}
									onChange={e => setIgnite(!ignite)} />
								Experiment Configuration & Ignition
							</p>
						</div>
						<form onSubmit={startRobot} className="mb-3">
							{ignite &&
								<div className="form-group">
									<label htmlFor="controllerSelector">Select Controller</label>
									<select
										className="form-control mb-3"
										id="controllerSelector"
										value={controller}
										onChange={handleControllerSelector}
										disabled={started}>
										<option value="false">None</option>
										<option value="true">The one I have</option>
									</select>
									<label htmlFor="identificationSelector">Select Identification</label>
									<select
										className="form-control mb-3"
										id="identificationSelector"
										disabled={started}>
										<option value="false">None</option>
									</select>
									<label htmlFor="telemetrySelector">Select values for telemetry</label>
									<select multiple
										className="form-control mb-3"
										id="telemetrySelector"
										disabled={started}>
										<option value="false">Position</option>
										<option value="false">RW Velocity</option>
										<option value="false">Shoulder Velocity</option>
										<option value="false">Elbow Velocity</option>
									</select>
									<label htmlFor="plotSelector">Select Plot</label>
									<select
										className="form-control mb-3"
										id="plotSelector"
										disabled={started}>
										<option value="false">None</option>
									</select>
								</div>
							}
							{!started &&
								<button
									type="submit"
									disabled={loading}
									className="btn btn-info shadow mb-0 w-100">
									{!loading &&
										<span>Start Robot</span>}
									{loading &&
										<span>Starting...</span>}
								</button>
							}
						</form>
					</div>
					<div className="content-section container shadow mb-6">
						<div className="border-bottom pt-1 mb-3">
							<p className="h6">
								<input
									className="mr-2"
									name="override"
									type="checkbox"
									value={override}
									onChange={e => setOverride(!override)} />
								Manual Override
							</p>
						</div>
						{override &&
							<div>
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
									<button disabled={(!started || !socketUp)} type="submit" className="btn btn-info  mb-2">Send Position X</button>
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
									<button disabled={(!started || !socketUp)} type="submit" className="btn btn-info  mb-2">Send Position Y</button>
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
									<button disabled={(!started || !socketUp)} type="submit" className="btn btn-info  mb-2">Send RW Velocity</button>
								</form>
								<form className="form-inline" onSubmit={handleShoulderPositionSubmit}>
									<div className="form-group mx-sm-3 mb-2">
										<label htmlFor="shoulder_pos"></label>
										<input
											className="form-control"
											name="shoulder_position"
											id="shoulder_pos"
											type="number"
											placeholder="Shoulder Position (rad)"
											value={shoulder_position}
											onChange={e => setShoulderPosition(e.target.value)} />
									</div>
									<button disabled={(!started || !socketUp)} type="submit" className="btn btn-info  mb-2">Send Shoulder Position</button>
								</form>
								<form className="form-inline" onSubmit={handleElbowPositionSubmit}>
									<div className="form-group mx-sm-3 mb-2">
										<label htmlFor="elbow_pos"></label>
										<input
											className="form-control"
											name="elbow_position"
											id="elbow_pos"
											type="number"
											placeholder="Elbow Position (rad)"
											value={elbow_position}
											onChange={e => setElbowPosition(e.target.value)} />
									</div>
									<button disabled={(!started || !socketUp)} type="submit" className="btn btn-info  mb-2">Send Elbow Position</button>
								</form>
							</div>
						}
					</div>
					<div className="content-section container shadow mb-6">
						<div className="border-bottom pt-1 mb-3">
							<p className="h6">
								<input
									className="mr-2"
									name="logs"
									type="checkbox"
									value={logs}
									onChange={e => setLogs(!logs)} />
								Log Screen
							</p>
						</div>
						{logs &&
							<div className="overflow-auto">
								<small className="float-left" style={{whiteSpace: 'pre-line'}}>
									{response}
								</small>
							</div>
						}
					</div>
				</div>
				
				<br/>
				
			</div>
		</div>
	);
}

export default withRouter(Control);
