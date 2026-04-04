import numpy as np
from mmwave.dataloader import DCA1000


def read_data(numFrames, raw_bytes, num_chirps, num_rx, num_samples) -> np.ndarray:

    adc_data_int16 = raw_bytes.view(dtype=np.int16).reshape(numFrames, -1)

    adc_data = np.apply_along_axis(
        DCA1000.organize,
        1,
        adc_data_int16,
        num_chirps,
        num_rx,
        num_samples,
    )

    print(f"初步数据维度: {adc_data.shape}")
    return adc_data
