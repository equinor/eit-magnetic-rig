import sys
import pandas as pd
import numpy as np

from joblib import load


def check_score_for_dataset(filename):
    df = pd.read_csv(filename, sep='\t', names=['Timestamp', 'Sample', 'PosTimestamp', 'PosX', 'PosY', 'PosZ',
                                                'ValTimestamp', 'A', 'B', 'C', 'Z'])
    df = df.fillna(0)

    print(df.tail())

    regX = load('models/PosXClassifier.joblib')
    print('Score for Position X', regX.score(df[['A', 'B', 'C', 'Z']], df['PosX']))

    regY = load('models/PosYClassifier.joblib')
    print('Score for Position Y', regY.score(df[['A', 'B', 'C', 'Z']], df['PosY']))

    regZ = load('models/PosZClassifier.joblib')
    print('Score for Position Z', regZ.score(df[['A', 'B', 'C', 'Z']], df['PosZ']))


def predict_position(sensors):
    features = np.array(list(map(float, sensors))).reshape((1, -1))

    regX = load('models/PosXClassifier.joblib')
    print('Prediction for Position X', regX.predict(features))

    regY = load('models/PosYClassifier.joblib')
    print('Prediction for Position Y', regY.predict(features))

    regZ = load('models/PosZClassifier.joblib')
    print('Prediction for Position Z', regZ.predict(features))


if __name__ == '__main__':
    """
        python rig_predictor.py dataset.csv ==> returns the score of the prediction models for this dataset.
        python rig_predictor.py 1472 423 6817 49 ==> returns the predicted position for this sensor value configuration.
    """
    if len(sys.argv) == 2:
        check_score_for_dataset(sys.argv[1])
    elif len(sys.argv) == 5:
        predict_position(sys.argv[1:])
    else:
        print('Invalid arguments!')
