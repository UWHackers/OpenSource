
/**
 * Module dependencies.
 */

var express = require('express');
var http = require('http');
var path = require('path');

var app = express();

// all environments
app.set('port', process.env.PORT || 3000);
app.use(express.json());
app.use(express.urlencoded());
app.use(express.methodOverride());
app.use(app.router);

// development only
if ('development' == app.get('env')) {
  app.use(express.errorHandler());
}

acronyms = {
  "BRB": "Be Right Back",
  "BTW": "By The Way",
  "BFF": "Best Friends Forever",
  "IMHO": "In My Humble Opinion",
  "IRL": "In Real Life",
  "JK": "Just Kidding",
  "LMAO": "Laughing My Ass Off",
  "LOL": "Living On Lipitor",
  "LASER": "Light amplification by the stimulated emission of radiation",
  "NATO": "The North Atlantic Treaty Organization",
  "WASP": "White anglo saxon protestant",
  "RSVP": "Répondez s'il vous plait"
};

app.get('/', function(req, res) {
    res.send(200, 'what up');
});
// app.get('/users', user.list);

http.createServer(app).listen(app.get('port'), function(){
  console.log('Express server listening on port ' + app.get('port'));
});
