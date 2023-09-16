import pandas as pd
import pickle
import numpy as np
import sys
data = sys.argv[1:]
with open('MLmodels/best_fertilizer_model.pkl', 'rb') as file:
    pickled_model = pickle.load(file)
column_names = ['Temperature', 'Humidity', 'Moisture', 'Soil Type', 'Crop Type', 'Nitrogen', 'Potassium', 'Phosphorus']
input_data = pd.DataFrame([data], columns=column_names)
input_data['Soil Type'] = pd.Categorical(input_data['Soil Type'])
input_data['Crop Type'] = pd.Categorical(input_data['Crop Type'])
input_data['Soil Type'] = input_data['Soil Type'].cat.codes
input_data['Crop Type'] = input_data['Crop Type'].cat.codes
processed_data = input_data.to_numpy()
predictions = pickled_model.predict(np.array(processed_data))
print(predictions[0])
