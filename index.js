const express = require("express");
const cors = require("cors");
const path = require("path");
const mongoose = require("mongoose");
const User = require("./models/User.js");
const TestCrops = require("./models/TestCrops.js");
const bcrypt = require("bcryptjs");
const jwt = require("jsonwebtoken");
const cookieParser = require("cookie-parser");
const { default: axios } = require("axios");
const { spawn } = require("child_process");
const app = express();
const bcryptSalt = bcrypt.genSaltSync(10);
const jwtSecret = "f8u230f20u203f20i3o";
const APIkey = "d19a1eac74129a54df64265af607e433";

const expressWs = require("express-ws");
expressWs(app);
let esp;
// let web_manual;
// const clientConnections = [];

let PH = [];
let TDS = [];
let Lux = [];
let Humidity = [];
let Temperature = [];
let HeatIndex = [];
let Corrected_PPM = [];
let DustDensity = [];
let RunningAverage = [];
let Nitrogen = [];
let Phosphorus = [];
let Potassium = [];
let pHsoil = [];
let EC = [];
let Tempsoil = [];
let Humsoil = [];

let latestPH;
let latestTDS;
let latestLux;
let latestHumidity;
let latestTemperature;
let latestHeatIndex;
let latestCorrected_PPM;
let latestDustDensity;
let latestRunningAverage;
let latestNitrogen;
let latestPhosphorus;
let latestPotassium;
let latestpHsoil;
let latestEC;
let latestTempsoil;
let latestHumsoil;
let rainfall;
let fertilizerRecommendation;
let cropRecommendation;
require("dotenv").config();
app.use(express.json());
app.use(cookieParser());
app.use(
  cors({
    credentials: true,
    // origin: ["http://localhost:4000"],
    origin: ["http://automaticcropcaretaker.com"],
  })
);
app.set("view engine", "ejs");
app.use(express.static(path.join(__dirname, "public")));

// WebSocket route for communication with ESP8266
app.ws("/ws", (ws, req) => {
  console.log("WebSocket connection established");
  // clientConnections.push(ws);
  const pingInterval = setInterval(() => {
    try {
      ws.send(JSON.stringify({ type: "ping" }));
    } catch (error) {
      console.log("Send ping error:", error);
    }
  }, 20000); //sending a ping every 20 seconds

  // Start a ping interval for each connection
  ws.on("message", (msg) => {
    console.log("Message from client:", msg);
    const data = JSON.parse(msg);
    if (data["type"] === "pong") {
      console.log(`Pong received from ${data["from"]}`);
      return;
    }
    if (data["type"] === "connection") {
      if (data["data"] === "ESP8266") {
        esp = ws;
      }
      //  else if (data["data"] === "website/manual") {
      //   web_manual = ws;
      // }
      return;
    }
    if (data["type"].indexOf("control") != -1) {
      esp.send(JSON.stringify(data));
    }
    if (data["type"] === "sensors_data") {
      latestPH = data["PH"];
      latestTDS = data["TDS"];
      latestLux = data["Lux"];
      latestHumidity = data["Humidity"];
      latestTemperature = data["Temperature"];
      latestHeatIndex = data["Heat Index"];
      latestCorrected_PPM = data["Corrected_PPM"];
      latestDustDensity = data["Dust Density"];
      latestRunningAverage = data["Running average"];
      latestNitrogen = data["Nitrogen"];
      latestPhosphorus = data["Phosphorus"];
      latestPotassium = data["Potassium"];
      latestpHsoil = data["pHsoil"];
      latestEC = data["EC"];
      latestTempsoil = data["Tempsoil"];
      latestHumsoil = data["Humsoil"];
      PH.push(data["PH"]);
      TDS.push(data["TDS"]);
      Lux.push(data["Lux"]);
      Humidity.push(data["Humidity"]);
      Temperature.push(data["Temperature"]);
      HeatIndex.push(data["Heat Index"]);
      Corrected_PPM.push(data["Corrected_PPM"]);
      DustDensity.push(data["Dust Density"]);
      RunningAverage.push(data["Running average"]);
      Nitrogen.push(data["Nitrogen"]);
      Phosphorus.push(data["Phosphorus"]);
      Potassium.push(data["Potassium"]);
      pHsoil.push(data["pHsoil"]);
      EC.push(data["EC"]);
      Tempsoil.push(data["Tempsoil"]);
      Humsoil.push(data["Humsoil"]);
      if (PH.length > 20) PH.shift();
      if (TDS.length > 20) TDS.shift();
      if (Lux.length > 20) Lux.shift();
      if (Humidity.length > 20) Humidity.shift();
      if (Temperature.length > 20) Temperature.shift();
      if (HeatIndex.length > 20) HeatIndex.shift();
      if (Corrected_PPM.length > 20) Corrected_PPM.shift();
      if (DustDensity.length > 20) DustDensity.shift();
      if (RunningAverage.length > 20) RunningAverage.shift();
      if (Nitrogen.length > 20) Nitrogen.shift();
      if (Phosphorus.length > 20) Phosphorus.shift();
      if (Potassium.length > 20) Potassium.shift();
      if (pHsoil.length > 20) pHsoil.shift();
      if (EC.length > 20) EC.shift();
      if (Tempsoil.length > 20) Tempsoil.shift();
      if (Humsoil.length > 20) Humsoil.shift();
    }
  });

  ws.on("close", () => {
    clearInterval(pingInterval);
    console.log("WebSocket connection closed");
    // const index = clientConnections.indexOf(ws);
    // if (index > -1) {
    //   clientConnections.splice(index, 1);
    // }
  });
});

// function sendEventToClients(eventData) {
//   for (const connection of clientConnections) {
//     connection.send(JSON.stringify(eventData));
//   }
// }

//register
app.get("/", (req, res) => {
  res.render("login.ejs");
});
app.post("/login", async (req, res) => {
  const { email, password } = req.body;
  console.log(email, password);
  const userDoc = await User.findOne({ email });
  if (userDoc) {
    const pass0k = bcrypt.compareSync(password, userDoc.password);
    if (pass0k) {
      jwt.sign(
        {
          email: userDoc.email,
          id: userDoc._id,
          firstname: userDoc.firstname,
          lastname: userDoc.lastname,
          address: userDoc.address,
        },
        jwtSecret,
        {},
        (err, token) => {
          if (err) throw err;
          res.cookie("token", token).json(userDoc);
          res.redirect("/dashboard");
        }
      );
    } else {
      res.status(422).json("pass not ok");
    }
  } else {
    res.status(404).json("Not found!");
  }
});
//login
app.get("/register", (req, res) => {
  res.render("register.ejs");
});
app.post("/register", async (req, res) => {
  const { firstname, lastname, address, email, password } = req.body;
  try {
    const userDoc = await User.create({
      firstname,
      lastname,
      address,
      email,
      password: bcrypt.hashSync(password, bcryptSalt),
    });
    res.json(userDoc);
    res.redirect("/");
  } catch (e) {
    res.status(422).json(e);
  }
});

//mongo
mongoose.connect(process.env.MONGOOSE_URL);
const connection = mongoose.connection;
connection.once("open", () => {
  console.log("MongoDB database connection established successfully");
});

//prediction
function predict(inputdata, callback) {
  const childPython = spawn("python", inputdata);
  let prediction = "";
  childPython.stdout.on("data", (data) => {
    console.log(`stdout: ${data}`);
    prediction += data;
  });
  childPython.stderr.on("data", (data) => {
    console.log(`stderr: ${data}`);
  });
  childPython.on("close", (code) => {
    console.log(`exited with code ${code}`);
    callback(prediction.trim());
  });
}

//dashboard
app.get("/dashboard", (req, res) => {
  const { token } = req.cookies;
  if (token) {
    jwt.verify(token, jwtSecret, {}, async (err, userData) => {
      if (err) throw err;
      const { firstname, lastname, email, _id } = await User.findById(
        userData.id
      );
      axios
        .get(
          `https://api.openweathermap.org/data/2.5/weather?q=${userData.address}&appid=${APIkey}`
        )
        .then((response) => {
          rainfall =
            response.data.rain * 4 ? response.data.rain["1h"] * 4 || 0 : 0;
        })
        .catch((err) => {
          console.error(err);
          res
            .status(500)
            .json({ error: "An error occurred while fetching weather data" });
        });
      res.render("index.ejs", { firstname, lastname, email, _id });
    });
  } else {
    res.redirect("/");
  }
});
app.get("/api/getDashboardData", (req, res) => {
  const { token } = req.cookies;
  if (token) {
    jwt.verify(token, jwtSecret, {}, async (err, userData) => {
      if (err) throw err;
      const data = {
        latestPH,
        latestTDS,
        latestLux,
        latestHumidity,
        latestTemperature,
        latestHeatIndex,
        latestCorrected_PPM,
        latestDustDensity,
        latestRunningAverage,
        latestNitrogen,
        latestPhosphorus,
        latestPotassium,
        latestpHsoil,
        latestEC,
        latestTempsoil,
        latestHumsoil,
        rainfall,
      };
      res.json(data);
    });
  } else {
    res.json(null);
  }
});
app.post("/api/getPredictionData", async (req, res) => {
  const { token } = req.cookies;
  console.log(token);
  const { soiltype, croptype } = req.body;
  if (token) {
    jwt.verify(token, jwtSecret, {}, async (err, userData) => {
      if (err) throw err;
      //going through an AI model
      console.log(soiltype, croptype);
      predict(
        [
          "crop.py",
          latestNitrogen,
          latestPhosphorus,
          latestPotassium,
          latestTemperature,
          latestHumidity,
          latestPH,
          rainfall,
        ],
        (prediction) => {
          console.log("Prediction:", prediction);
          cropRecommendation = prediction;
        }
      );
      predict(
        [
          "fertilizer.py",
          latestTemperature,
          latestHumidity,
          latestHumsoil,
          soiltype,
          croptype,
          latestNitrogen,
          latestPhosphorus,
          latestPotassium,
        ],
        (prediction) => {
          console.log("Prediction:", prediction);
          fertilizerRecommendation = prediction;
        }
      );
      let data = { cropRecommendation, fertilizerRecommendation };
      TestCrops.create({
        latestPH,
        latestTDS,
        latestLux,
        latestHumidity,
        latestTemperature,
        latestHeatIndex,
        latestCorrected_PPM,
        latestDustDensity,
        latestRunningAverage,
        latestNitrogen,
        latestPhosphorus,
        latestPotassium,
        latestpHsoil,
        latestEC,
        latestTempsoil,
        latestHumsoil,
        rainfall,
        soiltype,
        croptype,
        cropRecommendation,
        fertilizerRecommendation,
        author: userData.id,
      }).catch((err) => {
        console.error(err);
        res.status(500).json({
          error: "An error occurred while creating the test document",
        });
      });
      res.json(data);
    });
  } else {
    res.json(null);
  }
});

//charts
app.get("/charts", (req, res) => {
  const { token } = req.cookies;
  if (token) {
    jwt.verify(token, jwtSecret, {}, async (err, userData) => {
      const { id, firstname, lastname } = userData;
      res.render("charts.ejs", { firstname, lastname });
    });
  } else {
    res.json(null);
  }
});
app.get("/api/getChartData", (req, res) => {
  const { token } = req.cookies;
  if (token) {
    jwt.verify(token, jwtSecret, {}, async (err, userData) => {
      if (err) throw err;
      const data = {
        PH: 100,
        TDS,
        Lux,
        Humidity,
        Temperature,
        HeatIndex,
        Corrected_PPM,
        DustDensity,
        RunningAverage,
        Nitrogen,
        Phosphorus,
        Potassium,
        pHsoil,
        EC,
        Tempsoil,
        Humsoil,
      };
      res.json(data);
    });
  } else {
    res.redirect("/");
  }
});

//manual
app.get("/manual", (req, res) => {
  const { token } = req.cookies;
  if (token) {
    jwt.verify(token, jwtSecret, {}, async (err, userData) => {
      if (err) throw err;
      const { firstname, lastname, email, _id } = await User.findById(
        userData.id
      );
      res.render("manual.ejs", { firstname, lastname, email, _id });
    });
  } else {
    res.redirect("/");
  }
});

//tables
app.get("/tables", (req, res) => {
  const { token } = req.cookies;
  if (token) {
    jwt.verify(token, jwtSecret, {}, async (err, userData) => {
      const { id, firstname, lastname } = userData;
      const data = await TestCrops.find({ author: id });
      res.render("tables.ejs", { data, firstname, lastname });
    });
  } else {
    res.json(null);
  }
});

// app.post('/api/test', (req, res) => {
//   const { token } = req.cookies;
//   console.log(token);
//   const {pH, N,P,K} = req.body
//   if(token){
//     jwt.verify(token, jwtSecret, {}, async (err, userData) => {
//       if (err) throw err;
//       let recommendation
//         predict(['crop.py', N,P,K,latestTemperature,latestHumidity,pH,rainfall], (prediction) => {
//           console.log('Prediction:', prediction);
//           recommendation = prediction;
//           res.json([rainfall, latestHumidity, recommendation])
//       });
//     });
//   } else{
//   res.json(null)}
// });

app.post("/api/logout", (req, res) => {
  res.cookie("token", "").json(true);
});

app.listen(4000);
