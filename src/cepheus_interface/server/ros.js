// import packages
const express = require('express');
const fs = require("fs");
const bodyParser = require('body-parser');

const api = require('./api');

const app = express();
const port = 3000;

// app.use(function(req, res, next) {
// 	res.header("Access-Control-Allow-Origin", "http://localhost:xxxx");
// 	res.header("Access-Control-Allow-Headers", "Origin, X-Requested-With, Content-Type, Accept, Access-Control-Allow-Headers, Access-Control-Request-Method, Access-Control-Request-Headers, Authorization");
// 	res.header('Access-Control-Allow-Methods', 'GET, PUT, POST, DELETE, PATCH, OPTIONS');
// 	next();
// });

//Body Parsers middle ware
app.use(bodyParser.urlencoded({limit: '50mb', extended: true}));
app.use(bodyParser.json({limit: '50mb', extended: true}));

app.use('/', api);

app.listen(port, function(){
    console.log(`Server started on port ${port}`);
});
