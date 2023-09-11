# import pandas as pd
# import pickle
# import sys
# data = sys.argv[1:]
# with open('MLmodels/best_crop_prediction_model.pkl', 'rb') as file:
#     model = pickle.load(file)
# input_data = data
# column_names = ['N', 'P', 'K', 'temperature', 'humidity', 'ph', 'rainfall']
# df = pd.DataFrame([input_data], columns=column_names)
# predictions = model.predict(df)
# print(predictions[0])
