const mongoose = require("mongoose");
const Schema = mongoose.Schema;

const UserSchema = new Schema({
    firstname: {type: String, required: true}, 
    lastname: {type: String, required: true},
    address: {type: String, required: true},
    email: {type: String, unique: true, required: true},
    password: {type: String, required: true},
});

const UserModel = mongoose.model('User', UserSchema)

module.exports = UserModel;