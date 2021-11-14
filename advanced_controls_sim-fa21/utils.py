import csv
import numpy as np
import time
import scipy.io.wavfile as wavfile
# from pydub import AudioSegment

def read_csv(filename):
    return np.genfromtxt(filename, dtype=np.float, delimiter=",")


def train_test_split(data, split_ratio=0.7):
    """Splits data into training and test set according to the split_ratio.

    Arguments:
        data: dataset as a numpy array
        split_ratio: fraction of dataset to split as training data (must be between 0 and 1)

    Returns:
        Training Data (size = split_ratio * size of original dataset)
        Test Data (size = (1 - split_ratio) * size of original dataset)
    """
    train_data, test_data = data[:int(split_ratio *
                                      len(data)), :], data[int(split_ratio *
                                                               len(data)):, :]

    return train_data, test_data


def format_constant_energia(name, constant):
    # <Insert smug remark about left-pad>
    if len(name) < 37:
        padding = " " * (38 - len(name) - len("#define "))
    else:
        padding = "\t"
    return "#define {}{}{}".format(name, padding, constant)


def format_array_energia(name, array, dtype="float"):
    contents = ", ".join(map(str, array))
    return "{} {}[{}] = {{{}}};".format(dtype, name, len(array), contents)


def recording_timer(counts, record_time):
    timer = counts    # timer
    print("Start recording ...")
    time.sleep(1)
    while (timer != 0 ):
        timer=timer-1
        time.sleep(record_time)
        print(counts-timer)
    time.sleep(record_time)
    print("Recording is done.")

    
def sliding_mean(x, N):
    return np.convolve(x, np.ones((N,))/N)[(N-1):]


def read_wav(file, counts, res=16, points=200, mean_points=500, threshold=0.1, prelength=40):

    # read mp3
    frame_rate, a = wavfile.read(file)
    
    channels = a.shape[1]
    sample_rate = round(frame_rate/points*2)  # 2 seconds

    y = np.pad(a[:,0], (prelength, frame_rate*10), 'constant')
    
    # normalize data and clip the files
    length = int(y.shape[0] / sample_rate)
    N = length * sample_rate
    
    y_spl = np.split(y[:N], length)
    y_zero = y_spl - np.average(y_spl, axis=1)[:, None]
    y_sub = np.average(abs(y_zero), axis=1)


    start = 0
    y_max = max(y_sub)
    while y_sub[start] < y_max*threshold:
        start += 1
    start = start - prelength

    # save all data to numpy
    word_list = []
    for i in range(counts):
        word_list.append(y_sub[start+i*points: start+(i+1)*points]) 
    word_raw = np.array(word_list) 
    
    return word_raw
 

# NOT READ mp3 FILES, NEED INSTALL PACKAGES
# def read_mp3(file, counts, res=16, points=200, mean_points=500, threshold=0.1, prelength=40):

#     # read mp3
#     a = AudioSegment.from_mp3(file)
#     frame_rate = a.frame_rate
#     channels = a.channels
#     sample_width = a.sample_width
#     sample_rate = round(frame_rate/points*2)*channels  # 2 seconds

#     y = np.pad(np.array(a.get_array_of_samples()), (prelength, frame_rate*10), 'constant')
    
#     # normalize data and clip the files
#     length = int(y.shape[0] / sample_rate)
#     N = length * sample_rate
    
#     y_spl = np.split(y[:N], length)
#     y_zero = y_spl - np.average(y_spl, axis=1)[:, None]
#     y_sub = np.average(abs(y_zero), axis=1)


#     start = 0
#     y_max = max(y_sub)
#     while y_sub[start] < y_max*threshold:
#         start += 1
#     start = start - prelength

#     # save all data to numpy
#     word_list = []
#     for i in range(counts):
#         word_list.append(y_sub[start+i*points: start+(i+1)*points]) 
#     word_raw = np.array(word_list) 
    
#     return word_raw    