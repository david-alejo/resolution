% The first column is the spended time and the second is the cost. 
% If present, the third is the mean of the absolute value of differences between the original ETA and the ETA of the best individual. 
result = [ 62.2513 20.4183 ; 58.8675 20.399 ; 61.0134 20.3834 ; 62.9947 20.3757 ; 59.5571 20.4068 ; 60.9241 20.3962 ; 61.6057 20.3772 ; 60.4106 20.3766 ; 59.7349 20.4261 ; 60.1942 20.397 ]
evo_cost{1} = [21.4421 21.4421 21.3993 20.776 20.776 20.776 20.776 20.776 20.776 20.4955 20.4955 20.4955 20.4955 20.4955 20.4955 20.4955 20.4955 20.4955 20.4955 20.4955 20.4955 20.4955 20.4955 20.4955 20.4955 20.4955 20.4532 20.4532 20.4532 20.4532 20.4532 20.4491 20.4461 20.4461 20.4461 20.4461 20.4461 20.4461 20.4461 20.4447 20.4432 20.4423 20.4418 20.4417 20.4406 20.4399 20.4392 20.4375 20.4375 20.4375 20.4375 20.437 20.4356 20.4353 20.4351 20.435 20.4344 20.4341 20.4338 20.4335 20.4333 20.4332 20.4331 20.4329 20.4326 20.4324 20.4322 20.4322 20.4322 20.4322 20.4322 20.4322 20.4322 20.4321 20.4319 20.4319 20.4319 20.4319 20.4319 20.4318 20.4318 20.4318 20.4318 20.4318 20.4318 20.4318 20.4318 20.4318 20.4318 20.4318 20.4318 20.4318 20.4213 20.4213 20.4213 20.4213 20.4213 20.4213 20.4183 20.4183 ]
evo_time{1} = [1.44153 2.25784 3.01426 3.69335 4.39214 5.08815 5.73644 6.39867 7.08112 7.75854 8.40713 9.06147 9.72149 10.4105 11.0164 11.6213 12.2332 12.8233 13.4306 13.9979 14.5656 15.1549 15.7363 16.3262 16.9288 17.5809 18.2672 18.8952 19.4766 20.0369 20.6327 21.224 21.8208 22.3749 23.0395 23.6826 24.2748 24.893 25.5284 26.1553 26.7745 27.4175 28.0681 28.6944 29.3045 29.9102 30.4957 31.1915 31.8218 32.3831 32.9522 33.5317 34.1312 34.7161 35.3176 35.9049 36.4872 37.0851 37.6593 38.1948 38.7573 39.3252 39.9076 40.4931 41.095 41.7105 42.3215 42.9009 43.5047 44.1177 44.707 45.2852 45.8264 46.3944 47.0404 47.6863 48.335 48.9694 49.576 50.2563 50.8085 51.3436 51.9182 52.5124 53.1478 53.8576 54.5132 55.1383 55.7359 56.3008 56.8805 57.5114 58.1347 58.7464 59.329 59.9349 60.505 61.0939 61.6826 62.2478 ]
evo_cost{2} = [21.2549 21.2549 21.2549 21.2549 21.2549 21.0013 20.8554 20.8554 20.7419 20.7419 20.7419 20.7419 20.7419 20.7153 20.705 20.7034 20.6818 20.6599 20.6533 20.6502 20.6494 20.649 20.6465 20.5484 20.5173 20.5173 20.4889 20.4889 20.4889 20.4889 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 20.399 ]
evo_time{2} = [1.48042 2.3374 3.11666 3.92772 4.7175 5.46736 6.17495 6.85047 7.52808 8.21554 8.89285 9.52842 10.1755 10.8077 11.3778 11.9705 12.5761 13.2308 13.8636 14.4638 15.0489 15.6194 16.1724 16.7149 17.2541 17.7855 18.3318 18.8769 19.4193 19.9098 20.3792 20.8799 21.3936 21.9493 22.52 23.1243 23.6714 24.1815 24.6747 25.173 25.6437 26.131 26.6985 27.2553 27.7933 28.321 28.8217 29.3194 29.802 30.2891 30.8047 31.3795 31.961 32.5237 33.0722 33.6169 34.1514 34.6634 35.206 35.7603 36.3244 36.9153 37.496 38.047 38.5945 39.1158 39.6378 40.2121 40.7729 41.3318 41.8991 42.4547 43.0347 43.6004 44.1825 44.9013 45.5045 46.117 46.6874 47.1999 47.7482 48.3118 48.8894 49.5298 50.1218 50.7209 51.2776 51.8321 52.4326 52.969 53.5214 54.0845 54.6679 55.2558 55.8285 56.3881 57.0308 57.6551 58.2647 58.8645 ]
evo_cost{3} = [20.8805 20.8805 20.8805 20.8805 20.8272 20.7992 20.7915 20.647 20.647 20.647 20.647 20.647 20.647 20.6005 20.588 20.5233 20.4764 20.4745 20.4734 20.4726 20.4725 20.4724 20.4724 20.4723 20.4723 20.4723 20.4723 20.4723 20.4723 20.4723 20.4208 20.4208 20.4208 20.4208 20.4208 20.4208 20.4208 20.4208 20.4208 20.4208 20.4208 20.4208 20.4208 20.4208 20.4208 20.4208 20.4208 20.4208 20.4208 20.4208 20.4208 20.4208 20.4208 20.4208 20.4208 20.4208 20.4208 20.4087 20.4087 20.4087 20.4069 20.4035 20.402 20.3977 20.3938 20.3899 20.3861 20.3852 20.3851 20.385 20.385 20.385 20.385 20.385 20.385 20.3847 20.3844 20.3844 20.3844 20.3844 20.3842 20.3841 20.3841 20.3841 20.3841 20.3841 20.3841 20.3841 20.3841 20.3841 20.384 20.384 20.384 20.384 20.384 20.3837 20.3837 20.3837 20.3835 20.3834 ]
evo_time{3} = [1.59466 2.50155 3.33681 4.12048 4.90274 5.67607 6.36262 7.04391 7.74137 8.46116 9.16496 9.82384 10.4603 11.0788 11.6742 12.2273 12.8191 13.4305 14.0379 14.6252 15.2576 15.8586 16.4398 17.0313 17.6071 18.1997 18.7773 19.361 19.9712 20.5615 21.1684 21.7217 22.2905 22.8678 23.4041 23.937 24.5187 25.0677 25.6259 26.1832 26.7394 27.2629 27.816 28.3255 28.8654 29.4242 29.997 30.5898 31.1778 31.7361 32.2812 32.8285 33.3734 33.9354 34.5143 35.0897 35.6811 36.2881 36.8557 37.4487 38.0009 38.5585 39.1644 39.7603 40.3625 40.9716 41.6224 42.2137 42.7824 43.3298 43.9014 44.5298 45.1157 45.7496 46.3351 46.9198 47.4831 48.0928 48.642 49.2455 49.867 50.4374 50.9969 51.5466 52.139 52.7104 53.2744 53.8564 54.4732 55.1213 55.7071 56.2576 56.8356 57.4457 58.0408 58.6552 59.2527 59.8807 60.4718 61.009 ]
evo_cost{4} = [20.7752 20.7752 20.7752 20.7752 20.415 20.415 20.415 20.415 20.415 20.415 20.415 20.415 20.415 20.415 20.415 20.415 20.415 20.415 20.415 20.415 20.415 20.415 20.415 20.415 20.415 20.415 20.415 20.415 20.415 20.415 20.415 20.415 20.415 20.415 20.415 20.415 20.415 20.415 20.415 20.415 20.415 20.415 20.415 20.415 20.3829 20.3829 20.3829 20.3829 20.3829 20.3829 20.3829 20.3829 20.3829 20.3829 20.3829 20.3829 20.3829 20.3829 20.3829 20.3829 20.3829 20.3829 20.3826 20.3826 20.3826 20.3826 20.3826 20.3826 20.3826 20.3826 20.3826 20.3798 20.3798 20.3798 20.3798 20.3794 20.3794 20.3794 20.3794 20.3794 20.3794 20.3794 20.3794 20.3792 20.3757 20.3757 20.3757 20.3757 20.3757 20.3757 20.3757 20.3757 20.3757 20.3757 20.3757 20.3757 20.3757 20.3757 20.3757 20.3757 ]
evo_time{4} = [1.51665 2.42974 3.29967 4.07316 4.87212 5.63861 6.38123 7.09187 7.7941 8.47307 9.15061 9.80585 10.4689 11.1153 11.7783 12.4486 13.1656 13.7717 14.4019 15.0198 15.6114 16.2123 16.8002 17.4136 18.0291 18.6293 19.2281 19.8604 20.4574 21.0721 21.6815 22.2731 22.8457 23.4382 24.0627 24.6611 25.2788 25.9039 26.4986 27.1111 27.6939 28.3306 28.9698 29.5682 30.2501 31.197 31.8295 32.5313 33.1568 33.769 34.3602 34.9018 35.5622 36.1624 36.7028 37.2732 37.8941 38.5037 39.0423 39.6427 40.2696 40.8922 41.4598 42.1084 42.778 43.3566 43.9593 44.5621 45.0822 45.7154 46.3151 46.9037 47.4696 48.0668 48.6646 49.2552 49.8356 50.3904 50.9772 51.5634 52.1572 52.7632 53.3819 53.9938 54.5774 55.118 55.6953 56.3066 56.9305 57.4851 58.0246 58.6008 59.1835 59.7363 60.2558 60.7883 61.3487 61.8741 62.4462 62.9921 ]
evo_cost{5} = [20.9647 20.9647 20.9647 20.5865 20.5865 20.5865 20.5865 20.5865 20.5865 20.5865 20.5865 20.5865 20.5865 20.5594 20.5594 20.5594 20.5594 20.5594 20.5519 20.5519 20.5519 20.5519 20.5519 20.5519 20.5391 20.5391 20.5391 20.5391 20.4437 20.4437 20.4437 20.4437 20.4437 20.4437 20.4437 20.4437 20.4437 20.4437 20.4437 20.4437 20.4437 20.4437 20.4437 20.4437 20.4437 20.4437 20.4437 20.4437 20.4437 20.4437 20.4437 20.4437 20.4437 20.4437 20.4437 20.4437 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 20.4068 ]
evo_time{5} = [1.51182 2.35873 3.12227 3.88132 4.68094 5.49523 6.24448 6.98583 7.6593 8.37099 9.07321 9.75569 10.4261 11.1256 11.7529 12.4042 13.0089 13.5937 14.1684 14.6967 15.2455 15.8352 16.4143 16.9825 17.562 18.1061 18.6278 19.2351 19.7815 20.3071 20.8696 21.4659 22.02 22.5822 23.1499 23.715 24.2567 24.7973 25.3162 25.8337 26.3861 26.9228 27.4982 28.1124 28.7129 29.2519 29.7641 30.2774 30.7967 31.325 31.88 32.4334 32.9984 33.5655 34.1265 34.6722 35.1874 35.7008 36.2347 36.764 37.2928 37.8502 38.4223 38.9775 39.5289 40.0773 40.6448 41.2092 41.7811 42.3423 42.9205 43.4847 44.0353 44.5677 45.1319 45.7072 46.2974 46.8745 47.4428 48.016 48.586 49.1187 49.6547 50.2477 50.8566 51.4403 52.0474 52.6526 53.2454 53.8274 54.3536 54.9215 55.535 56.1341 56.746 57.3281 57.914 58.4617 59.0069 59.5545 ]
evo_cost{6} = [21.7558 20.6302 20.6302 20.6302 20.6302 20.6302 20.6302 20.6302 20.5668 20.555 20.54 20.54 20.4886 20.4886 20.4886 20.4886 20.4886 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 20.3962 ]
evo_time{6} = [1.45027 2.38155 3.27938 4.1156 4.90194 5.68935 6.40347 7.09873 7.7764 8.4416 9.11468 9.78482 10.4391 11.0646 11.6663 12.2284 12.7443 13.2827 13.8437 14.4751 15.0827 15.6958 16.28 16.8621 17.392 17.9259 18.4633 19.0359 19.6247 20.2317 20.7939 21.3593 21.909 22.4429 22.9921 23.5498 24.122 24.704 25.2895 25.9024 26.4923 27.0568 27.5928 28.122 28.6753 29.2323 29.7975 30.3725 30.9588 31.5107 32.0389 32.5522 33.1131 33.6696 34.2226 34.8049 35.3889 35.938 36.502 37.0554 37.6149 38.1751 38.728 39.3138 39.9083 40.4946 41.0704 41.7035 42.3011 42.8828 43.4794 44.0762 44.656 45.2717 45.8544 46.4269 47.0041 47.5596 48.1221 48.7009 49.3282 49.9371 50.5306 51.128 51.6887 52.242 52.8136 53.4427 54.0916 54.7179 55.3285 55.9354 56.5721 57.2341 57.8217 58.4167 59.0322 59.681 60.3153 60.9192 ]
evo_cost{7} = [20.7062 20.7062 20.7062 20.7062 20.7062 20.7062 20.7062 20.7062 20.7062 20.7062 20.584 20.584 20.584 20.5344 20.5344 20.5344 20.5157 20.4659 20.4239 20.4105 20.4098 20.4094 20.4093 20.4093 20.4092 20.4092 20.4092 20.4092 20.4092 20.4092 20.4092 20.4092 20.4092 20.4092 20.4092 20.4092 20.4092 20.4086 20.4086 20.4086 20.4063 20.4032 20.4026 20.4014 20.3994 20.399 20.3897 20.3849 20.3816 20.3816 20.3816 20.3816 20.3816 20.3813 20.381 20.3807 20.3807 20.3807 20.3807 20.3807 20.3806 20.3806 20.3805 20.3805 20.3805 20.3805 20.3805 20.3805 20.3805 20.3805 20.3805 20.3805 20.3805 20.3805 20.3805 20.3805 20.3805 20.3805 20.3805 20.3805 20.3805 20.3805 20.3805 20.3805 20.3805 20.3805 20.3805 20.3805 20.3805 20.3805 20.3805 20.3805 20.379 20.3779 20.3772 20.3772 20.3772 20.3772 20.3772 20.3772 ]
evo_time{7} = [1.5417 2.48612 3.32036 4.12369 4.92521 5.74727 6.52317 7.2325 7.89709 8.58449 9.22091 9.87334 10.4702 11.0856 11.731 12.3679 13.0144 13.6359 14.3054 14.8952 15.5322 16.1268 16.7682 17.4404 18.0906 18.6946 19.2361 19.7462 20.2898 20.8381 21.4304 22.0524 22.6679 23.2555 23.807 24.3473 24.9045 25.4818 26.1173 26.7308 27.3339 27.9224 28.4835 29.0206 29.5534 30.1432 30.7157 31.3583 32.0071 32.6179 33.248 33.83 34.3599 34.9408 35.4958 36.1205 36.7735 37.4115 38.1047 38.739 39.3183 39.8149 40.3641 40.9439 41.5445 42.1653 42.8356 43.3993 43.9789 44.507 45.0495 45.5904 46.1569 46.7399 47.3493 47.9211 48.4878 49.0595 49.63 50.176 50.7329 51.3434 51.9474 52.4905 53.0504 53.6031 54.1569 54.6989 55.2393 55.7769 56.3605 56.95 57.5092 58.0535 58.5883 59.1399 59.7154 60.3111 60.9004 61.6023 ]
evo_cost{8} = [21.04 21.04 21.04 20.7728 20.7728 20.7728 20.7728 20.7728 20.6834 20.4968 20.4968 20.4968 20.4968 20.4968 20.4968 20.4968 20.4968 20.4968 20.4968 20.4968 20.4968 20.4968 20.4968 20.4968 20.4875 20.4688 20.4603 20.4539 20.4504 20.4483 20.4475 20.4467 20.4461 20.4455 20.4453 20.4451 20.445 20.445 20.445 20.445 20.445 20.445 20.4055 20.4055 20.3999 20.3999 20.3999 20.3999 20.3999 20.3999 20.3999 20.3999 20.3923 20.3923 20.3923 20.3923 20.3923 20.3923 20.3923 20.3923 20.3923 20.3923 20.3923 20.3923 20.3923 20.3923 20.3923 20.3923 20.3923 20.3923 20.3923 20.3923 20.3923 20.3923 20.3923 20.3923 20.3923 20.3923 20.3923 20.3923 20.3923 20.3766 20.3766 20.3766 20.3766 20.3766 20.3766 20.3766 20.3766 20.3766 20.3766 20.3766 20.3766 20.3766 20.3766 20.3766 20.3766 20.3766 20.3766 20.3766 ]
evo_time{8} = [1.6227 2.54335 3.4229 4.23696 5.03549 5.83189 6.57747 7.23687 7.88044 8.56507 9.28092 9.94944 10.6145 11.2563 11.872 12.4892 13.062 13.6508 14.2478 14.8658 15.4788 16.0857 16.6891 17.3224 17.888 18.4236 19.0017 19.5874 20.2104 20.8437 21.483 22.0804 22.6368 23.1937 23.7693 24.3538 24.9526 25.5449 26.1342 26.7571 27.3631 27.9589 28.5258 29.0553 29.5826 30.1162 30.6649 31.2273 31.8231 32.4035 32.941 33.4884 33.9985 34.4811 35.0107 35.5599 36.1068 36.6726 37.2396 37.8088 38.3875 38.9126 39.4322 39.9855 40.5376 41.1149 41.6816 42.2529 42.7968 43.3292 43.8925 44.4709 45.0384 45.6053 46.1503 46.7284 47.297 47.8418 48.3948 48.9726 49.559 50.1429 50.707 51.3056 51.8572 52.3899 52.9319 53.4857 54.0756 54.6427 55.1974 55.7896 56.3638 56.9473 57.5345 58.0912 58.6792 59.2506 59.8101 60.4079 ]
evo_cost{9} = [20.8379 20.8379 20.8379 20.8379 20.8379 20.8379 20.8379 20.7063 20.6416 20.5951 20.5951 20.4998 20.4614 20.449 20.4443 20.4421 20.4415 20.4412 20.4409 20.4408 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4406 20.4343 20.4343 20.4343 20.4343 20.4343 20.4343 20.4261 20.4261 20.4261 20.4261 20.4261 20.4261 20.4261 20.4261 20.4261 20.4261 20.4261 20.4261 20.4261 20.4261 20.4261 20.4261 20.4261 20.4261 20.4261 ]
evo_time{9} = [1.5604 2.54033 3.40017 4.21228 5.00191 5.70702 6.36105 7.04021 7.73909 8.45453 9.14577 9.80365 10.4141 10.9942 11.5372 12.0546 12.6169 13.2579 13.9008 14.5246 15.1529 15.7765 16.3532 16.9322 17.482 18.0348 18.5909 19.1575 19.7074 20.2628 20.7828 21.2992 21.8157 22.3501 22.9007 23.4597 24.0214 24.5618 25.091 25.6289 26.1531 26.6689 27.1808 27.7142 28.2555 28.7836 29.3181 29.8417 30.3869 30.9334 31.4785 32.011 32.5299 33.0584 33.645 34.2371 34.7904 35.3436 35.9159 36.571 37.1454 37.6736 38.2347 38.8491 39.3991 39.9567 40.5462 41.1432 41.7161 42.2477 42.8165 43.3614 43.9241 44.5052 45.1438 45.7781 46.3435 46.9248 47.4894 48.0632 48.6423 49.2299 49.8242 50.4169 51.0526 51.6619 52.2859 52.8794 53.4941 54.0987 54.6606 55.2169 55.7789 56.419 57.0461 57.6202 58.1757 58.6925 59.2177 59.7321 ]
evo_cost{10} = [21.4697 21.4697 20.9302 20.9302 20.9302 20.9302 20.9302 20.9302 20.6261 20.6261 20.6261 20.6261 20.6261 20.6261 20.6221 20.5934 20.5701 20.5607 20.5549 20.5513 20.5478 20.5461 20.5461 20.5461 20.5461 20.5461 20.5461 20.5461 20.5453 20.5441 20.5441 20.5418 20.5408 20.5406 20.5406 20.5378 20.4716 20.4688 20.4688 20.4688 20.4688 20.4688 20.4688 20.4688 20.4688 20.4634 20.4634 20.4524 20.4524 20.4524 20.4524 20.4524 20.4456 20.4456 20.4456 20.4456 20.4456 20.4456 20.4456 20.4376 20.4376 20.4376 20.4376 20.4376 20.4376 20.4376 20.4358 20.4358 20.4358 20.4358 20.4358 20.4358 20.435 20.434 20.434 20.4334 20.4334 20.4333 20.4332 20.4323 20.4318 20.4316 20.4316 20.4316 20.4316 20.4316 20.4316 20.4316 20.4316 20.4316 20.4316 20.4316 20.4316 20.4316 20.4316 20.4316 20.4316 20.4316 20.397 20.397 ]
evo_time{10} = [1.51574 2.38711 3.19207 3.94863 4.7238 5.47719 6.21196 6.93798 7.64057 8.32698 9.00883 9.66115 10.3006 10.9322 11.5699 12.2173 12.832 13.4561 14.0535 14.65 15.2426 15.8467 16.435 17.0036 17.5936 18.1851 18.7918 19.382 19.989 20.6 21.2016 21.7899 22.3654 22.9642 23.5731 24.1953 24.7592 25.2883 25.8155 26.3989 26.9963 27.5828 28.1652 28.7651 29.3515 29.9065 30.4567 30.9866 31.5463 32.1554 32.7419 33.3325 33.893 34.4381 34.9687 35.4918 36.0488 36.6232 37.2098 37.7835 38.3383 38.8963 39.4371 39.968 40.5208 41.0522 41.6117 42.1613 42.7256 43.2845 43.8443 44.4022 44.9475 45.5254 46.0923 46.6354 47.2152 47.781 48.3339 48.9136 49.5356 50.1163 50.7035 51.2764 51.8279 52.4081 53.0341 53.619 54.2171 54.7555 55.2768 55.7838 56.3209 56.8532 57.4092 57.9819 58.563 59.1257 59.6687 60.1916 ]