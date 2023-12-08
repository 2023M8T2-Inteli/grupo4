const express = require('express');
const cors = require('cors');
const orders = require('./routes/queue');
const users = require('./routes/users');
const tools = require('./routes/tools');
const points = require('./routes/points');
const speech = require('./routes/speech');


const app = express();
app.use(cors());
const PORT = process.env.PORT || 5000;

// Middleware
app.use(express.json());

// Mount routes
app.use('/orders', orders);
app.use('/users', users);
app.use('/tools', tools);
app.use('/points', points);
app.use('/speech', speech);

// Start the server
app.listen(PORT, () => {
    console.log(`Server is running on http://localhost:${PORT}`);
});
