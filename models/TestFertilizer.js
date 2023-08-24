const mongoose = require('mongoose');

const testFertilizerSchema = new mongoose.Schema({
    N: Number,
    P: Number,
    K: Number,
    pH: Number,
    temperature: Number,
    humidity: Number,
    soiltype: String,
    moisture: Number,
    croptype: String,
    recommendation: String,
    author: {type: mongoose.Schema.Types.ObjectId, ref:'User'},
});

const TestFertilizerModel = mongoose.model('TestFertilizer', testFertilizerSchema);

module.exports = TestFertilizerModel;