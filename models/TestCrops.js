const mongoose = require('mongoose');

const testCropSchema = new mongoose.Schema({
    latestPH: Number,
    latestTDS: Number,
    latestLux: Number,
    latestHumidity: Number,
    latestTemperature: Number,
    latestHeatIndex: Number,
    latestCorrected_PPM: Number,
    latestDustDensity: Number,
    latestRunningAverage: Number,
    latestNitrogen: Number,
    latestPhosphorus: Number,
    latestPotassium: Number,
    latestpHsoil: Number,
    latestEC: Number,
    latestTempsoil: Number,
    latestHumsoil: Number,
    rainfall: Number,
    soiltype: String,
    croptype: String,
    cropRecommendation: String,
    fertilizerRecommendation: String,
    author: {type: mongoose.Schema.Types.ObjectId, ref:'User'},
});

const TestCropModel = mongoose.model('TestCrop', testCropSchema);

module.exports = TestCropModel;