#include <inttypes.h>
/* sine wavetable 4096 x 4096 */
uint16_t sine[4096] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 12, 12, 12, 13, 13, 13, 14, 14, 15, 15, 15, 16, 16, 16, 17, 17, 18, 18, 19, 19, 19, 20, 20, 21, 21, 22, 22, 23, 23, 24, 24, 25, 25, 26, 26, 27, 27, 28, 28, 29, 29, 30, 30, 31, 31, 32, 32, 33, 34, 34, 35, 35, 36, 36, 37, 38, 38, 39, 39, 40, 41, 41, 42, 43, 43, 44, 45, 45, 46, 47, 47, 48, 49, 49, 50, 51, 51, 52, 53, 54, 54, 55, 56, 56, 57, 58, 59, 59, 60, 61, 62, 62, 63, 64, 65, 66, 66, 67, 68, 69, 70, 70, 71, 72, 73, 74, 75, 75, 76, 77, 78, 79, 80, 81, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 135, 136, 137, 138, 139, 140, 141, 143, 144, 145, 146, 147, 148, 150, 151, 152, 153, 154, 155, 157, 158, 159, 160, 162, 163, 164, 165, 166, 168, 169, 170, 171, 173, 174, 175, 177, 178, 179, 180, 182, 183, 184, 186, 187, 188, 190, 191, 192, 194, 195, 196, 198, 199, 200, 202, 203, 204, 206, 207, 208, 210, 211, 213, 214, 215, 217, 218, 220, 221, 223, 224, 225, 227, 228, 230, 231, 233, 234, 236, 237, 238, 240, 241, 243, 244, 246, 247, 249, 250, 252, 253, 255, 256, 258, 260, 261, 263, 264, 266, 267, 269, 270, 272, 273, 275, 277, 278, 280, 281, 283, 285, 286, 288, 289, 291, 293, 294, 296, 298, 299, 301, 302, 304, 306, 307, 309, 311, 312, 314, 316, 317, 319, 321, 322, 324, 326, 328, 329, 331, 333, 334, 336, 338, 340, 341, 343, 345, 347, 348, 350, 352, 354, 355, 357, 359, 361, 362, 364, 366, 368, 370, 371, 373, 375, 377, 379, 381, 382, 384, 386, 388, 390, 392, 393, 395, 397, 399, 401, 403, 405, 406, 408, 410, 412, 414, 416, 418, 420, 422, 424, 425, 427, 429, 431, 433, 435, 437, 439, 441, 443, 445, 447, 449, 451, 453, 455, 457, 459, 461, 463, 465, 467, 469, 471, 473, 475, 477, 479, 481, 483, 485, 487, 489, 491, 493, 495, 497, 499, 501, 503, 505, 507, 509, 511, 514, 516, 518, 520, 522, 524, 526, 528, 530, 532, 535, 537, 539, 541, 543, 545, 547, 549, 552, 554, 556, 558, 560, 562, 565, 567, 569, 571, 573, 575, 578, 580, 582, 584, 586, 589, 591, 593, 595, 597, 600, 602, 604, 606, 609, 611, 613, 615, 618, 620, 622, 624, 627, 629, 631, 633, 636, 638, 640, 642, 645, 647, 649, 652, 654, 656, 659, 661, 663, 665, 668, 670, 672, 675, 677, 679, 682, 684, 686, 689, 691, 694, 696, 698, 701, 703, 705, 708, 710, 713, 715, 717, 720, 722, 724, 727, 729, 732, 734, 736, 739, 741, 744, 746, 749, 751, 753, 756, 758, 761, 763, 766, 768, 771, 773, 776, 778, 780, 783, 785, 788, 790, 793, 795, 798, 800, 803, 805, 808, 810, 813, 815, 818, 820, 823, 825, 828, 830, 833, 835, 838, 841, 843, 846, 848, 851, 853, 856, 858, 861, 864, 866, 869, 871, 874, 876, 879, 882, 884, 887, 889, 892, 894, 897, 900, 902, 905, 907, 910, 913, 915, 918, 921, 923, 926, 928, 931, 934, 936, 939, 942, 944, 947, 950, 952, 955, 958, 960, 963, 966, 968, 971, 974, 976, 979, 982, 984, 987, 990, 992, 995, 998, 1000, 1003, 1006, 1009, 1011, 1014, 1017, 1019, 1022, 1025, 1028, 1030, 1033, 1036, 1039, 1041, 1044, 1047, 1049, 1052, 1055, 1058, 1060, 1063, 1066, 1069, 1071, 1074, 1077, 1080, 1083, 1085, 1088, 1091, 1094, 1096, 1099, 1102, 1105, 1108, 1110, 1113, 1116, 1119, 1122, 1124, 1127, 1130, 1133, 1136, 1138, 1141, 1144, 1147, 1150, 1153, 1155, 1158, 1161, 1164, 1167, 1170, 1172, 1175, 1178, 1181, 1184, 1187, 1189, 1192, 1195, 1198, 1201, 1204, 1207, 1209, 1212, 1215, 1218, 1221, 1224, 1227, 1230, 1232, 1235, 1238, 1241, 1244, 1247, 1250, 1253, 1256, 1259, 1261, 1264, 1267, 1270, 1273, 1276, 1279, 1282, 1285, 1288, 1291, 1293, 1296, 1299, 1302, 1305, 1308, 1311, 1314, 1317, 1320, 1323, 1326, 1329, 1332, 1335, 1337, 1340, 1343, 1346, 1349, 1352, 1355, 1358, 1361, 1364, 1367, 1370, 1373, 1376, 1379, 1382, 1385, 1388, 1391, 1394, 1397, 1400, 1403, 1406, 1409, 1412, 1415, 1418, 1421, 1424, 1427, 1430, 1433, 1436, 1439, 1442, 1445, 1448, 1451, 1454, 1457, 1460, 1463, 1466, 1469, 1472, 1475, 1478, 1481, 1484, 1487, 1490, 1493, 1496, 1499, 1502, 1505, 1508, 1511, 1514, 1517, 1520, 1523, 1526, 1529, 1532, 1535, 1538, 1541, 1544, 1547, 1551, 1554, 1557, 1560, 1563, 1566, 1569, 1572, 1575, 1578, 1581, 1584, 1587, 1590, 1593, 1596, 1599, 1603, 1606, 1609, 1612, 1615, 1618, 1621, 1624, 1627, 1630, 1633, 1636, 1639, 1642, 1646, 1649, 1652, 1655, 1658, 1661, 1664, 1667, 1670, 1673, 1676, 1679, 1683, 1686, 1689, 1692, 1695, 1698, 1701, 1704, 1707, 1710, 1714, 1717, 1720, 1723, 1726, 1729, 1732, 1735, 1738, 1741, 1745, 1748, 1751, 1754, 1757, 1760, 1763, 1766, 1769, 1773, 1776, 1779, 1782, 1785, 1788, 1791, 1794, 1798, 1801, 1804, 1807, 1810, 1813, 1816, 1819, 1822, 1826, 1829, 1832, 1835, 1838, 1841, 1844, 1847, 1851, 1854, 1857, 1860, 1863, 1866, 1869, 1873, 1876, 1879, 1882, 1885, 1888, 1891, 1894, 1898, 1901, 1904, 1907, 1910, 1913, 1916, 1920, 1923, 1926, 1929, 1932, 1935, 1938, 1941, 1945, 1948, 1951, 1954, 1957, 1960, 1963, 1967, 1970, 1973, 1976, 1979, 1982, 1985, 1989, 1992, 1995, 1998, 2001, 2004, 2007, 2011, 2014, 2017, 2020, 2023, 2026, 2029, 2033, 2036, 2039, 2042, 2045, 2048, 2051, 2055, 2058, 2061, 2064, 2067, 2070, 2073, 2077, 2080, 2083, 2086, 2089, 2092, 2095, 2099, 2102, 2105, 2108, 2111, 2114, 2117, 2121, 2124, 2127, 2130, 2133, 2136, 2139, 2143, 2146, 2149, 2152, 2155, 2158, 2161, 2164, 2168, 2171, 2174, 2177, 2180, 2183, 2186, 2190, 2193, 2196, 2199, 2202, 2205, 2208, 2212, 2215, 2218, 2221, 2224, 2227, 2230, 2233, 2237, 2240, 2243, 2246, 2249, 2252, 2255, 2258, 2262, 2265, 2268, 2271, 2274, 2277, 2280, 2283, 2287, 2290, 2293, 2296, 2299, 2302, 2305, 2308, 2312, 2315, 2318, 2321, 2324, 2327, 2330, 2333, 2336, 2340, 2343, 2346, 2349, 2352, 2355, 2358, 2361, 2364, 2367, 2371, 2374, 2377, 2380, 2383, 2386, 2389, 2392, 2395, 2398, 2402, 2405, 2408, 2411, 2414, 2417, 2420, 2423, 2426, 2429, 2432, 2436, 2439, 2442, 2445, 2448, 2451, 2454, 2457, 2460, 2463, 2466, 2469, 2473, 2476, 2479, 2482, 2485, 2488, 2491, 2494, 2497, 2500, 2503, 2506, 2509, 2512, 2515, 2519, 2522, 2525, 2528, 2531, 2534, 2537, 2540, 2543, 2546, 2549, 2552, 2555, 2558, 2561, 2564, 2567, 2570, 2573, 2576, 2579, 2582, 2586, 2589, 2592, 2595, 2598, 2601, 2604, 2607, 2610, 2613, 2616, 2619, 2622, 2625, 2628, 2631, 2634, 2637, 2640, 2643, 2646, 2649, 2652, 2655, 2658, 2661, 2664, 2667, 2670, 2673, 2676, 2679, 2682, 2685, 2688, 2691, 2694, 2697, 2700, 2703, 2706, 2709, 2712, 2715, 2718, 2721, 2724, 2727, 2729, 2732, 2735, 2738, 2741, 2744, 2747, 2750, 2753, 2756, 2759, 2762, 2765, 2768, 2771, 2774, 2777, 2780, 2783, 2785, 2788, 2791, 2794, 2797, 2800, 2803, 2806, 2809, 2812, 2815, 2818, 2821, 2823, 2826, 2829, 2832, 2835, 2838, 2841, 2844, 2847, 2850, 2852, 2855, 2858, 2861, 2864, 2867, 2870, 2873, 2875, 2878, 2881, 2884, 2887, 2890, 2893, 2896, 2898, 2901, 2904, 2907, 2910, 2913, 2916, 2918, 2921, 2924, 2927, 2930, 2933, 2935, 2938, 2941, 2944, 2947, 2950, 2952, 2955, 2958, 2961, 2964, 2966, 2969, 2972, 2975, 2978, 2980, 2983, 2986, 2989, 2992, 2994, 2997, 3000, 3003, 3006, 3008, 3011, 3014, 3017, 3019, 3022, 3025, 3028, 3030, 3033, 3036, 3039, 3041, 3044, 3047, 3050, 3052, 3055, 3058, 3061, 3063, 3066, 3069, 3072, 3074, 3077, 3080, 3082, 3085, 3088, 3090, 3093, 3096, 3099, 3101, 3104, 3107, 3109, 3112, 3115, 3117, 3120, 3123, 3125, 3128, 3131, 3133, 3136, 3139, 3141, 3144, 3147, 3149, 3152, 3155, 3157, 3160, 3163, 3165, 3168, 3170, 3173, 3176, 3178, 3181, 3184, 3186, 3189, 3191, 3194, 3197, 3199, 3202, 3204, 3207, 3210, 3212, 3215, 3217, 3220, 3223, 3225, 3228, 3230, 3233, 3235, 3238, 3240, 3243, 3246, 3248, 3251, 3253, 3256, 3258, 3261, 3263, 3266, 3268, 3271, 3273, 3276, 3278, 3281, 3283, 3286, 3288, 3291, 3293, 3296, 3298, 3301, 3303, 3306, 3308, 3311, 3313, 3316, 3318, 3321, 3323, 3326, 3328, 3331, 3333, 3335, 3338, 3340, 3343, 3345, 3348, 3350, 3352, 3355, 3357, 3360, 3362, 3365, 3367, 3369, 3372, 3374, 3377, 3379, 3381, 3384, 3386, 3388, 3391, 3393, 3396, 3398, 3400, 3403, 3405, 3407, 3410, 3412, 3414, 3417, 3419, 3421, 3424, 3426, 3428, 3431, 3433, 3435, 3438, 3440, 3442, 3445, 3447, 3449, 3451, 3454, 3456, 3458, 3461, 3463, 3465, 3467, 3470, 3472, 3474, 3476, 3479, 3481, 3483, 3485, 3488, 3490, 3492, 3494, 3496, 3499, 3501, 3503, 3505, 3508, 3510, 3512, 3514, 3516, 3519, 3521, 3523, 3525, 3527, 3529, 3532, 3534, 3536, 3538, 3540, 3542, 3545, 3547, 3549, 3551, 3553, 3555, 3557, 3559, 3562, 3564, 3566, 3568, 3570, 3572, 3574, 3576, 3578, 3580, 3583, 3585, 3587, 3589, 3591, 3593, 3595, 3597, 3599, 3601, 3603, 3605, 3607, 3609, 3611, 3613, 3615, 3617, 3619, 3621, 3623, 3625, 3627, 3629, 3631, 3633, 3635, 3637, 3639, 3641, 3643, 3645, 3647, 3649, 3651, 3653, 3655, 3657, 3659, 3661, 3663, 3665, 3667, 3669, 3670, 3672, 3674, 3676, 3678, 3680, 3682, 3684, 3686, 3688, 3689, 3691, 3693, 3695, 3697, 3699, 3701, 3703, 3704, 3706, 3708, 3710, 3712, 3714, 3715, 3717, 3719, 3721, 3723, 3724, 3726, 3728, 3730, 3732, 3733, 3735, 3737, 3739, 3741, 3742, 3744, 3746, 3748, 3749, 3751, 3753, 3755, 3756, 3758, 3760, 3761, 3763, 3765, 3767, 3768, 3770, 3772, 3773, 3775, 3777, 3778, 3780, 3782, 3783, 3785, 3787, 3788, 3790, 3792, 3793, 3795, 3797, 3798, 3800, 3802, 3803, 3805, 3806, 3808, 3810, 3811, 3813, 3814, 3816, 3818, 3819, 3821, 3822, 3824, 3825, 3827, 3829, 3830, 3832, 3833, 3835, 3836, 3838, 3839, 3841, 3842, 3844, 3845, 3847, 3848, 3850, 3851, 3853, 3854, 3856, 3857, 3859, 3860, 3862, 3863, 3865, 3866, 3867, 3869, 3870, 3872, 3873, 3875, 3876, 3877, 3879, 3880, 3882, 3883, 3884, 3886, 3887, 3889, 3890, 3891, 3893, 3894, 3895, 3897, 3898, 3899, 3901, 3902, 3903, 3905, 3906, 3907, 3909, 3910, 3911, 3913, 3914, 3915, 3917, 3918, 3919, 3920, 3922, 3923, 3924, 3925, 3927, 3928, 3929, 3930, 3932, 3933, 3934, 3935, 3937, 3938, 3939, 3940, 3941, 3943, 3944, 3945, 3946, 3947, 3948, 3950, 3951, 3952, 3953, 3954, 3955, 3957, 3958, 3959, 3960, 3961, 3962, 3963, 3964, 3965, 3967, 3968, 3969, 3970, 3971, 3972, 3973, 3974, 3975, 3976, 3977, 3978, 3979, 3980, 3981, 3983, 3984, 3985, 3986, 3987, 3988, 3989, 3990, 3991, 3992, 3993, 3994, 3995, 3996, 3996, 3997, 3998, 3999, 4000, 4001, 4002, 4003, 4004, 4005, 4006, 4007, 4008, 4009, 4010, 4010, 4011, 4012, 4013, 4014, 4015, 4016, 4017, 4017, 4018, 4019, 4020, 4021, 4022, 4023, 4023, 4024, 4025, 4026, 4027, 4027, 4028, 4029, 4030, 4031, 4031, 4032, 4033, 4034, 4034, 4035, 4036, 4037, 4037, 4038, 4039, 4040, 4040, 4041, 4042, 4043, 4043, 4044, 4045, 4045, 4046, 4047, 4047, 4048, 4049, 4049, 4050, 4051, 4051, 4052, 4053, 4053, 4054, 4055, 4055, 4056, 4056, 4057, 4058, 4058, 4059, 4059, 4060, 4061, 4061, 4062, 4062, 4063, 4063, 4064, 4065, 4065, 4066, 4066, 4067, 4067, 4068, 4068, 4069, 4069, 4070, 4070, 4071, 4071, 4072, 4072, 4073, 4073, 4074, 4074, 4074, 4075, 4075, 4076, 4076, 4077, 4077, 4077, 4078, 4078, 4079, 4079, 4080, 4080, 4080, 4081, 4081, 4081, 4082, 4082, 4082, 4083, 4083, 4084, 4084, 4084, 4085, 4085, 4085, 4085, 4086, 4086, 4086, 4087, 4087, 4087, 4088, 4088, 4088, 4088, 4089, 4089, 4089, 4089, 4090, 4090, 4090, 4090, 4091, 4091, 4091, 4091, 4091, 4092, 4092, 4092, 4092, 4092, 4092, 4093, 4093, 4093, 4093, 4093, 4093, 4094, 4094, 4094, 4094, 4094, 4094, 4094, 4094, 4094, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4094, 4094, 4094, 4094, 4094, 4094, 4094, 4094, 4094, 4093, 4093, 4093, 4093, 4093, 4093, 4092, 4092, 4092, 4092, 4092, 4092, 4091, 4091, 4091, 4091, 4091, 4090, 4090, 4090, 4090, 4089, 4089, 4089, 4089, 4088, 4088, 4088, 4088, 4087, 4087, 4087, 4086, 4086, 4086, 4085, 4085, 4085, 4085, 4084, 4084, 4084, 4083, 4083, 4082, 4082, 4082, 4081, 4081, 4081, 4080, 4080, 4080, 4079, 4079, 4078, 4078, 4077, 4077, 4077, 4076, 4076, 4075, 4075, 4074, 4074, 4074, 4073, 4073, 4072, 4072, 4071, 4071, 4070, 4070, 4069, 4069, 4068, 4068, 4067, 4067, 4066, 4066, 4065, 4065, 4064, 4063, 4063, 4062, 4062, 4061, 4061, 4060, 4059, 4059, 4058, 4058, 4057, 4056, 4056, 4055, 4055, 4054, 4053, 4053, 4052, 4051, 4051, 4050, 4049, 4049, 4048, 4047, 4047, 4046, 4045, 4045, 4044, 4043, 4043, 4042, 4041, 4040, 4040, 4039, 4038, 4037, 4037, 4036, 4035, 4034, 4034, 4033, 4032, 4031, 4031, 4030, 4029, 4028, 4027, 4027, 4026, 4025, 4024, 4023, 4023, 4022, 4021, 4020, 4019, 4018, 4017, 4017, 4016, 4015, 4014, 4013, 4012, 4011, 4010, 4010, 4009, 4008, 4007, 4006, 4005, 4004, 4003, 4002, 4001, 4000, 3999, 3998, 3997, 3996, 3996, 3995, 3994, 3993, 3992, 3991, 3990, 3989, 3988, 3987, 3986, 3985, 3984, 3983, 3981, 3980, 3979, 3978, 3977, 3976, 3975, 3974, 3973, 3972, 3971, 3970, 3969, 3968, 3967, 3965, 3964, 3963, 3962, 3961, 3960, 3959, 3958, 3957, 3955, 3954, 3953, 3952, 3951, 3950, 3948, 3947, 3946, 3945, 3944, 3943, 3941, 3940, 3939, 3938, 3937, 3935, 3934, 3933, 3932, 3930, 3929, 3928, 3927, 3925, 3924, 3923, 3922, 3920, 3919, 3918, 3917, 3915, 3914, 3913, 3911, 3910, 3909, 3907, 3906, 3905, 3903, 3902, 3901, 3899, 3898, 3897, 3895, 3894, 3893, 3891, 3890, 3889, 3887, 3886, 3884, 3883, 3882, 3880, 3879, 3877, 3876, 3875, 3873, 3872, 3870, 3869, 3867, 3866, 3865, 3863, 3862, 3860, 3859, 3857, 3856, 3854, 3853, 3851, 3850, 3848, 3847, 3845, 3844, 3842, 3841, 3839, 3838, 3836, 3835, 3833, 3832, 3830, 3829, 3827, 3825, 3824, 3822, 3821, 3819, 3818, 3816, 3814, 3813, 3811, 3810, 3808, 3806, 3805, 3803, 3802, 3800, 3798, 3797, 3795, 3793, 3792, 3790, 3788, 3787, 3785, 3783, 3782, 3780, 3778, 3777, 3775, 3773, 3772, 3770, 3768, 3767, 3765, 3763, 3761, 3760, 3758, 3756, 3755, 3753, 3751, 3749, 3748, 3746, 3744, 3742, 3741, 3739, 3737, 3735, 3733, 3732, 3730, 3728, 3726, 3724, 3723, 3721, 3719, 3717, 3715, 3714, 3712, 3710, 3708, 3706, 3704, 3703, 3701, 3699, 3697, 3695, 3693, 3691, 3689, 3688, 3686, 3684, 3682, 3680, 3678, 3676, 3674, 3672, 3670, 3669, 3667, 3665, 3663, 3661, 3659, 3657, 3655, 3653, 3651, 3649, 3647, 3645, 3643, 3641, 3639, 3637, 3635, 3633, 3631, 3629, 3627, 3625, 3623, 3621, 3619, 3617, 3615, 3613, 3611, 3609, 3607, 3605, 3603, 3601, 3599, 3597, 3595, 3593, 3591, 3589, 3587, 3585, 3583, 3580, 3578, 3576, 3574, 3572, 3570, 3568, 3566, 3564, 3562, 3559, 3557, 3555, 3553, 3551, 3549, 3547, 3545, 3542, 3540, 3538, 3536, 3534, 3532, 3529, 3527, 3525, 3523, 3521, 3519, 3516, 3514, 3512, 3510, 3508, 3505, 3503, 3501, 3499, 3496, 3494, 3492, 3490, 3488, 3485, 3483, 3481, 3479, 3476, 3474, 3472, 3470, 3467, 3465, 3463, 3461, 3458, 3456, 3454, 3451, 3449, 3447, 3445, 3442, 3440, 3438, 3435, 3433, 3431, 3428, 3426, 3424, 3421, 3419, 3417, 3414, 3412, 3410, 3407, 3405, 3403, 3400, 3398, 3396, 3393, 3391, 3388, 3386, 3384, 3381, 3379, 3377, 3374, 3372, 3369, 3367, 3365, 3362, 3360, 3357, 3355, 3352, 3350, 3348, 3345, 3343, 3340, 3338, 3335, 3333, 3331, 3328, 3326, 3323, 3321, 3318, 3316, 3313, 3311, 3308, 3306, 3303, 3301, 3298, 3296, 3293, 3291, 3288, 3286, 3283, 3281, 3278, 3276, 3273, 3271, 3268, 3266, 3263, 3261, 3258, 3256, 3253, 3251, 3248, 3246, 3243, 3240, 3238, 3235, 3233, 3230, 3228, 3225, 3223, 3220, 3217, 3215, 3212, 3210, 3207, 3204, 3202, 3199, 3197, 3194, 3191, 3189, 3186, 3184, 3181, 3178, 3176, 3173, 3170, 3168, 3165, 3163, 3160, 3157, 3155, 3152, 3149, 3147, 3144, 3141, 3139, 3136, 3133, 3131, 3128, 3125, 3123, 3120, 3117, 3115, 3112, 3109, 3107, 3104, 3101, 3099, 3096, 3093, 3090, 3088, 3085, 3082, 3080, 3077, 3074, 3071, 3069, 3066, 3063, 3061, 3058, 3055, 3052, 3050, 3047, 3044, 3041, 3039, 3036, 3033, 3030, 3028, 3025, 3022, 3019, 3017, 3014, 3011, 3008, 3006, 3003, 3000, 2997, 2994, 2992, 2989, 2986, 2983, 2980, 2978, 2975, 2972, 2969, 2966, 2964, 2961, 2958, 2955, 2952, 2950, 2947, 2944, 2941, 2938, 2935, 2933, 2930, 2927, 2924, 2921, 2918, 2916, 2913, 2910, 2907, 2904, 2901, 2898, 2896, 2893, 2890, 2887, 2884, 2881, 2878, 2875, 2873, 2870, 2867, 2864, 2861, 2858, 2855, 2852, 2850, 2847, 2844, 2841, 2838, 2835, 2832, 2829, 2826, 2823, 2821, 2818, 2815, 2812, 2809, 2806, 2803, 2800, 2797, 2794, 2791, 2788, 2785, 2783, 2780, 2777, 2774, 2771, 2768, 2765, 2762, 2759, 2756, 2753, 2750, 2747, 2744, 2741, 2738, 2735, 2732, 2729, 2727, 2724, 2721, 2718, 2715, 2712, 2709, 2706, 2703, 2700, 2697, 2694, 2691, 2688, 2685, 2682, 2679, 2676, 2673, 2670, 2667, 2664, 2661, 2658, 2655, 2652, 2649, 2646, 2643, 2640, 2637, 2634, 2631, 2628, 2625, 2622, 2619, 2616, 2613, 2610, 2607, 2604, 2601, 2598, 2595, 2592, 2589, 2586, 2582, 2579, 2576, 2573, 2570, 2567, 2564, 2561, 2558, 2555, 2552, 2549, 2546, 2543, 2540, 2537, 2534, 2531, 2528, 2525, 2522, 2519, 2515, 2512, 2509, 2506, 2503, 2500, 2497, 2494, 2491, 2488, 2485, 2482, 2479, 2476, 2473, 2469, 2466, 2463, 2460, 2457, 2454, 2451, 2448, 2445, 2442, 2439, 2436, 2432, 2429, 2426, 2423, 2420, 2417, 2414, 2411, 2408, 2405, 2402, 2398, 2395, 2392, 2389, 2386, 2383, 2380, 2377, 2374, 2371, 2367, 2364, 2361, 2358, 2355, 2352, 2349, 2346, 2343, 2340, 2336, 2333, 2330, 2327, 2324, 2321, 2318, 2315, 2312, 2308, 2305, 2302, 2299, 2296, 2293, 2290, 2287, 2283, 2280, 2277, 2274, 2271, 2268, 2265, 2262, 2258, 2255, 2252, 2249, 2246, 2243, 2240, 2237, 2233, 2230, 2227, 2224, 2221, 2218, 2215, 2212, 2208, 2205, 2202, 2199, 2196, 2193, 2190, 2186, 2183, 2180, 2177, 2174, 2171, 2168, 2164, 2161, 2158, 2155, 2152, 2149, 2146, 2143, 2139, 2136, 2133, 2130, 2127, 2124, 2121, 2117, 2114, 2111, 2108, 2105, 2102, 2099, 2095, 2092, 2089, 2086, 2083, 2080, 2077, 2073, 2070, 2067, 2064, 2061, 2058, 2055, 2051, 2048, 2045, 2042, 2039, 2036, 2033, 2029, 2026, 2023, 2020, 2017, 2014, 2011, 2007, 2004, 2001, 1998, 1995, 1992, 1989, 1985, 1982, 1979, 1976, 1973, 1970, 1967, 1963, 1960, 1957, 1954, 1951, 1948, 1945, 1941, 1938, 1935, 1932, 1929, 1926, 1923, 1920, 1916, 1913, 1910, 1907, 1904, 1901, 1898, 1894, 1891, 1888, 1885, 1882, 1879, 1876, 1873, 1869, 1866, 1863, 1860, 1857, 1854, 1851, 1847, 1844, 1841, 1838, 1835, 1832, 1829, 1826, 1822, 1819, 1816, 1813, 1810, 1807, 1804, 1801, 1798, 1794, 1791, 1788, 1785, 1782, 1779, 1776, 1773, 1769, 1766, 1763, 1760, 1757, 1754, 1751, 1748, 1745, 1741, 1738, 1735, 1732, 1729, 1726, 1723, 1720, 1717, 1714, 1710, 1707, 1704, 1701, 1698, 1695, 1692, 1689, 1686, 1683, 1679, 1676, 1673, 1670, 1667, 1664, 1661, 1658, 1655, 1652, 1649, 1646, 1642, 1639, 1636, 1633, 1630, 1627, 1624, 1621, 1618, 1615, 1612, 1609, 1606, 1603, 1599, 1596, 1593, 1590, 1587, 1584, 1581, 1578, 1575, 1572, 1569, 1566, 1563, 1560, 1557, 1554, 1551, 1547, 1544, 1541, 1538, 1535, 1532, 1529, 1526, 1523, 1520, 1517, 1514, 1511, 1508, 1505, 1502, 1499, 1496, 1493, 1490, 1487, 1484, 1481, 1478, 1475, 1472, 1469, 1466, 1463, 1460, 1457, 1454, 1451, 1448, 1445, 1442, 1439, 1436, 1433, 1430, 1427, 1424, 1421, 1418, 1415, 1412, 1409, 1406, 1403, 1400, 1397, 1394, 1391, 1388, 1385, 1382, 1379, 1376, 1373, 1370, 1367, 1364, 1361, 1358, 1355, 1352, 1349, 1346, 1343, 1340, 1337, 1335, 1332, 1329, 1326, 1323, 1320, 1317, 1314, 1311, 1308, 1305, 1302, 1299, 1296, 1293, 1291, 1288, 1285, 1282, 1279, 1276, 1273, 1270, 1267, 1264, 1261, 1259, 1256, 1253, 1250, 1247, 1244, 1241, 1238, 1235, 1232, 1230, 1227, 1224, 1221, 1218, 1215, 1212, 1209, 1207, 1204, 1201, 1198, 1195, 1192, 1189, 1187, 1184, 1181, 1178, 1175, 1172, 1170, 1167, 1164, 1161, 1158, 1155, 1153, 1150, 1147, 1144, 1141, 1138, 1136, 1133, 1130, 1127, 1124, 1122, 1119, 1116, 1113, 1110, 1108, 1105, 1102, 1099, 1096, 1094, 1091, 1088, 1085, 1083, 1080, 1077, 1074, 1071, 1069, 1066, 1063, 1060, 1058, 1055, 1052, 1049, 1047, 1044, 1041, 1039, 1036, 1033, 1030, 1028, 1025, 1022, 1019, 1017, 1014, 1011, 1009, 1006, 1003, 1000, 998, 995, 992, 990, 987, 984, 982, 979, 976, 974, 971, 968, 966, 963, 960, 958, 955, 952, 950, 947, 944, 942, 939, 936, 934, 931, 928, 926, 923, 921, 918, 915, 913, 910, 907, 905, 902, 900, 897, 894, 892, 889, 887, 884, 882, 879, 876, 874, 871, 869, 866, 864, 861, 858, 856, 853, 851, 848, 846, 843, 841, 838, 835, 833, 830, 828, 825, 823, 820, 818, 815, 813, 810, 808, 805, 803, 800, 798, 795, 793, 790, 788, 785, 783, 780, 778, 776, 773, 771, 768, 766, 763, 761, 758, 756, 753, 751, 749, 746, 744, 741, 739, 736, 734, 732, 729, 727, 724, 722, 720, 717, 715, 713, 710, 708, 705, 703, 701, 698, 696, 694, 691, 689, 686, 684, 682, 679, 677, 675, 672, 670, 668, 665, 663, 661, 659, 656, 654, 652, 649, 647, 645, 642, 640, 638, 636, 633, 631, 629, 627, 624, 622, 620, 618, 615, 613, 611, 609, 606, 604, 602, 600, 597, 595, 593, 591, 589, 586, 584, 582, 580, 578, 575, 573, 571, 569, 567, 565, 562, 560, 558, 556, 554, 552, 549, 547, 545, 543, 541, 539, 537, 535, 532, 530, 528, 526, 524, 522, 520, 518, 516, 514, 511, 509, 507, 505, 503, 501, 499, 497, 495, 493, 491, 489, 487, 485, 483, 481, 479, 477, 475, 473, 471, 469, 467, 465, 463, 461, 459, 457, 455, 453, 451, 449, 447, 445, 443, 441, 439, 437, 435, 433, 431, 429, 427, 425, 424, 422, 420, 418, 416, 414, 412, 410, 408, 406, 405, 403, 401, 399, 397, 395, 393, 392, 390, 388, 386, 384, 382, 381, 379, 377, 375, 373, 371, 370, 368, 366, 364, 362, 361, 359, 357, 355, 354, 352, 350, 348, 347, 345, 343, 341, 340, 338, 336, 334, 333, 331, 329, 328, 326, 324, 322, 321, 319, 317, 316, 314, 312, 311, 309, 307, 306, 304, 302, 301, 299, 298, 296, 294, 293, 291, 289, 288, 286, 285, 283, 281, 280, 278, 277, 275, 273, 272, 270, 269, 267, 266, 264, 263, 261, 260, 258, 256, 255, 253, 252, 250, 249, 247, 246, 244, 243, 241, 240, 238, 237, 236, 234, 233, 231, 230, 228, 227, 225, 224, 223, 221, 220, 218, 217, 215, 214, 213, 211, 210, 208, 207, 206, 204, 203, 202, 200, 199, 198, 196, 195, 194, 192, 191, 190, 188, 187, 186, 184, 183, 182, 180, 179, 178, 177, 175, 174, 173, 171, 170, 169, 168, 166, 165, 164, 163, 162, 160, 159, 158, 157, 155, 154, 153, 152, 151, 150, 148, 147, 146, 145, 144, 143, 141, 140, 139, 138, 137, 136, 135, 133, 132, 131, 130, 129, 128, 127, 126, 125, 124, 122, 121, 120, 119, 118, 117, 116, 115, 114, 113, 112, 111, 110, 109, 108, 107, 106, 105, 104, 103, 102, 101, 100, 99, 98, 97, 96, 95, 94, 93, 92, 91, 90, 90, 89, 88, 87, 86, 85, 84, 83, 82, 81, 81, 80, 79, 78, 77, 76, 75, 75, 74, 73, 72, 71, 70, 70, 69, 68, 67, 66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 59, 58, 57, 56, 56, 55, 54, 54, 53, 52, 51, 51, 50, 49, 49, 48, 47, 47, 46, 45, 45, 44, 43, 43, 42, 41, 41, 40, 39, 39, 38, 38, 37, 36, 36, 35, 35, 34, 34, 33, 32, 32, 31, 31, 30, 30, 29, 29, 28, 28, 27, 27, 26, 26, 25, 25, 24, 24, 23, 23, 22, 22, 21, 21, 20, 20, 19, 19, 19, 18, 18, 17, 17, 16, 16, 16, 15, 15, 15, 14, 14, 13, 13, 13, 12, 12, 12, 11, 11, 11, 10, 10, 10, 9, 9, 9, 8, 8, 8, 8, 7, 7, 7, 7, 6, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  };
