
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
  "RSVP": "Répondez s'il vous plait",
  "REST": "Representational state transfer",
  "API": "Application programming interface"
};

app.get('/', function(req, res) {
    res.send(200, {
        'message': 'Sudo Acronyms API v1'
    });
});

app.get('/:acronym', function(req, res) {
    var acronym = req.params.acronym.toUpperCase();
    console.log(acronym);

    var result = {};
    result[acronym] = acronyms[acronym];

    if (result[acronym]) {
        res.send(200, result);
    } else {
        res.send(400, {
            'error': 'acronym not found'
        });
    }
});

app.post('/:acronym/:full', function(req, res) {
    var acronym = req.params.acronym.toUpperCase();
    var full = req.params.full;

    if (acronyms[acronym]) {
        res.send(400, {'error' : 'acronym exists'});
    } else {
        acronyms[acronym] = full;
        res.send(200, {'status': 'done'})
    }
});

http.createServer(app).listen(app.get('port'), function(){
  console.log('Express server listening on port ' + app.get('port'));
});
