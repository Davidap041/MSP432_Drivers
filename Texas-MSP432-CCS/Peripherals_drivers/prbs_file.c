/*
 * prbs_file.c
 *
 *  Created on: 2 de out de 2020
 *      Author: davia
 */

float prbs[13] = { 0, 1.5, 1, 0, -1, -1.5, 0, 0, 0, 0, 0, 0, 0 };
//float calibration_signal[13] = { 0, 1.5, -1.5, 0, 1, -1, 0, 0.5, -0.5, 0.2, -0.2,
//                                 1, -1 };
//float calibration_signal[31] = { 0, 0, 0, 0, 1, -1, 0, 0.5, -0.5, 0.2,
//                                 -0.2, 1, -1, -0.5, -0.4, -0.3, -0.2, -0.1, 0,
//                                 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8};
float calibration_signal[62] = { -1.5, -1.4, -1.3, -1.2, -1.1, -1, -0.9, -0.8,
                                 -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0,
                                 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9,
                                 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.4, 1.3, 1.2,
                                 1.1, 1, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2,
                                 0.1, 0, -0.1, -0.2, -0.3, -0.4, -0.5, -0.6,
                                 -0.7, -0.8, -0.9, -1, -1.1, -1.2, -1.3, -1.4,
                                 -1.5 };
