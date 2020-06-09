import React from 'react';
import { Switch, Route } from 'react-router-dom';
import './App.css';

import Control from './components/Control';

function App() {
	return (
		<div className="App">
			<Switch>
				<Route exact path="/" component={Control} />
			</Switch>
		</div>
	);
}

export default App;
