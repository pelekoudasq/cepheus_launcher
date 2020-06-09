import React, { useState, useEffect } from 'react';
import { withRouter } from 'react-router';
// import { Link } from 'react-router-dom';
import io from 'socket.io-client';


function Control() {

	const [response, setResponse] = useState("");
	const [started, setStarted] = useState("");
	const [loading, setLoading] = useState("");

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


	function startRobot(e) {

		e.preventDefault();
		setLoading(true);

		let requestOptions = {
			method: 'POST'
		};

		fetch(`http://localhost:9000/start`, requestOptions);
	};

	function stopRobot(e) {

		e.preventDefault();
		setLoading(true);

		let requestOptions = {
			method: 'POST'
		};

		fetch(`http://localhost:9000/stop`, requestOptions);
	}

	return (
		<div>
			<div style={{ textAlign: 'center' }}>
				<p>Cepheus</p>
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
			<br/>
			<div className="content-section">
				<samp className="float-left" style={{whiteSpace: 'pre-line'}}>
					{response}
				</samp>
			</div>
		</div>
	);
}

export default withRouter(Control);
