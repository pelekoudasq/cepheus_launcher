import React from 'react';
import { Switch, Route } from 'react-router-dom';
import './App.css';

import Control from './components/Control';

function App() {
	return (
		<div className="App">
			<div className="row container-fluid">
				<Switch>
					<Route exact path="/" component={Control} />
				</Switch>
			</div>
		</div>
	);
}

export default App;
