const mongoose = require('mongoose');

const testCropSchema = new mongoose.Schema({
    N: Number,
    P: Number,
    K: Number,
    pH: Number,
    moisture: Number,
    temperature: Number,
    humidity: Number,
    rainfall: Number,
    soiltype: String,
    croptype: String,
    recommendation: String,
    author: {type: mongoose.Schema.Types.ObjectId, ref:'User'},
});

const TestCropModel = mongoose.model('TestCrop', testCropSchema);

module.exports = TestCropModel;