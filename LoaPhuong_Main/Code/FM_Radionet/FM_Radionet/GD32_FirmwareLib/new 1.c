static void ProcessNewBlackboxData(void)
{
    esp32RxMessageTimeout = 0;
    DEBUG_PRINTF("ESP32 message size %d: %s\r\n", BlackBoxBuffer.BufferIndex, BlackBoxBuffer.Buffer);

    /** Xu ly ban tin tu ESP32 */
    /** 4G,MODE=<FM/4G/MIC/NONE/IDLE>,FREQ_RUN=<10270>,FREQ1=<>,FREQ2=<>,FREQ3=<>,VOL=<70>,THRESHOLD=<90>,CRC=12345#
		* Dinh dang gia tri tan so: = Freq (Hz) /10000, vi du: 102.7MHz = 102700000 /10000 = 10270 
	* Lệnh ghi/đọc trực tiếp thanh ghi của KT0935R: KT_WRITE<xxx>=<yy> trong đó: xxx là địa chỉ thanh ghi, 3 digit, yy: giá trị cần ghi
	* KT_READ<xxx>
	*/
    
    if (strstr((char *)BlackBoxBuffer.Buffer, "TEST=1"))
    {
        xSystem.Parameter.enter_test_mode = 1;
    }
    
    if (xSystem.Parameter.enter_test_mode)
    {
        static uint32_t delay = 5;
        if (delay)
        {
            delay--;
            FM_SetVolume(100);
        }
    }
            
    if (strstr((char *)BlackBoxBuffer.Buffer, "4G,") && BlackBoxBuffer.BufferIndex > 10)
    {
        /* Check CRC: ,CRC=12345# */
        char *crc = strstr((char *)BlackBoxBuffer.Buffer, "CRC=");
        if (crc)
        {
            esp_die_cnt = 0;
            uint16_t crc16 = GetNumberFromString(4, crc);

            /* Tinh CRC thuc cua chuoi: Tru 10 ki tu cuoi CRC=12345# */
            uint16_t crcCal = CalculateCRC16(BlackBoxBuffer.Buffer, BlackBoxBuffer.BufferIndex - 10);

            if (crc16 != crcCal)
            {
                DEBUG_PRINTF("\rCRC failed: %u - %u", crc16, crcCal);
                return;
            }
        }
        else
            return;

        /* So config can luu */
        uint8_t configNum = 0;

        /* Split các truong du lieu */
        char *mToken = strtok((char *)BlackBoxBuffer.Buffer, ",");

        /** Duyet cac truong */
        while (mToken != NULL)
        {
            //			DEBUG_PRINTF("\rToken: %s", mToken);

            /** Lệnh reset */
            if (strstr(mToken, "RESET=1"))
            {
                NVIC_SystemReset();
            }

            /** Check Master Mode */
            if (strstr(mToken, "MODE=FM"))
            {
                xSystem.Parameter.Mode = MODE_FM_REMOTE;
                goto NEXT_TOKEN;
            }
            if (strstr(mToken, "MODE=INTERNET"))
            {
                xSystem.Parameter.Mode = MODE_INTERNET;
                goto NEXT_TOKEN;
            }
            if (strstr(mToken, "MODE=MIC"))
            {
                xSystem.Parameter.Mode = MODE_MIC;
                goto NEXT_TOKEN;
            }
            if (strstr(mToken, "MODE=NONE"))
            { /* Khong cho phep hoat dong */
                if (xSystem.Parameter.Mode != MODE_NONE)
                {
                    xSystem.Parameter.Mode = MODE_NONE;
                    configNum++;
                }
                goto NEXT_TOKEN;
            }
            if (strstr(mToken, "MODE=IDLE"))
            {
                xSystem.Parameter.Mode = MODE_IDLE;
                goto NEXT_TOKEN;
            }

            /** Check tan so cai dat: FREQ1,...,FREQ5 */
            if (strstr(mToken, "FREQ1="))
            {
                uint16_t freq = GetNumberFromString(6, mToken);
                if (freq != xSystem.Parameter.Frequency[0])
                {
                    xSystem.Parameter.Frequency[0] = freq;
                    configNum++;
                }
                goto NEXT_TOKEN;
            }
            if (strstr(mToken, "FREQ2="))
            {
                uint16_t freq = GetNumberFromString(6, mToken);
                if (freq != xSystem.Parameter.Frequency[1])
                {
                    xSystem.Parameter.Frequency[1] = freq;
                    configNum++;
                }
                goto NEXT_TOKEN;
            }
            if (strstr(mToken, "FREQ3="))
            {
                uint16_t freq = GetNumberFromString(6, mToken);
                if (freq != xSystem.Parameter.Frequency[2])
                {
                    xSystem.Parameter.Frequency[2] = freq;
                    configNum++;
                }
                goto NEXT_TOKEN;
            }

            /** Check tan so lam viec: FREQ */
            if (strstr(mToken, "FREQ_RUN="))
            {
                uint16_t Freq = GetNumberFromString(9, mToken);
                uint8_t isFreqValid = isValidFreq(Freq);
                if (isFreqValid != 0xFF && Freq != xSystem.Parameter.FreqRun)
                {
                    xSystem.Parameter.FreqRun = Freq;
                    xSystem.Parameter.FreqIndex = isFreqValid;

                    /* Dieu khien module thu tan so moi */
                    if (xSystem.Parameter.Mode == MODE_FM_REMOTE)
                    {
                        /* Setup Freq & volume */
                        KT093x_User_FMTune(xSystem.Parameter.FreqRun); //MHz x100
                        FM_SetVolume(xSystem.Parameter.RemoteVolume);

                        /* Hien thi tan so dang phat */
                        Led7_DispFrequency(Freq);
                    }
                }
                goto NEXT_TOKEN;
            }

            /** Check Volume cai dat cho FM */
            if (strstr(mToken, "VOL="))
            {
                uint8_t volume = GetNumberFromString(4, mToken);
                if (xSystem.Parameter.RemoteVolume != volume)
                {
                    xSystem.Parameter.RemoteVolume = volume;

                    /* Thiet lap volume cho module FM */
                    FM_SetVolume(xSystem.Parameter.RemoteVolume);
                }
                goto NEXT_TOKEN;
            }

            /** Điều khiển local mode về IDLE ngay lập tức */
            if (strstr(mToken, "LOCAL_MODE=IDLE"))
            {
                xSystem.Parameter.ModeLocal = MODE_IDLE;
                goto NEXT_TOKEN;
            }

            /** Lệnh Read/Write thanh ghi KT0935 : KT_WRITE<xxx>=yy */
            if (strstr(mToken, "KT_WRITE"))
            {
                uint8_t reg = GetNumberFromString(8, mToken);
                uint8_t value = GetNumberFromString(12, mToken);

                //Add to write list
                if (KT_Write_List.total == 0)
                {
                    KT_Write_List.List[0].reg = reg;
                    KT_Write_List.List[0].value = value;
                    KT_Write_List.List[0].isWrite = 1;
                    KT_Write_List.total++;
                }
                else
                {
                    //Tìm vị trí trống
                    for (uint8_t i = 0; i < KT_REG_LIST_MAX; i++)
                    {
                        if (KT_Write_List.List[i].isWrite == 0)
                        {
                            KT_Write_List.List[i].reg = reg;
                            KT_Write_List.List[i].value = value;
                            KT_Write_List.List[i].isWrite = 1;
                            KT_Write_List.total++;
                            break;
                        }
                    }
                }
                goto NEXT_TOKEN;
            }

            if (strstr(mToken, "KT_READ"))
            {
                uint8_t reg = GetNumberFromString(7, mToken);

                //Add to write list
                if (KT_Read_List.total == 0)
                {
                    KT_Read_List.List[0].reg = reg;
                    KT_Read_List.List[0].value = 0;
                    KT_Read_List.List[0].isRead = 1;
                    KT_Read_List.List[0].isReadDone = 0;
                    KT_Read_List.total++;
                }
                else
                {
                    //Tìm vị trí trống
                    for (uint8_t i = 0; i < KT_REG_LIST_MAX; i++)
                    {
                        if (KT_Read_List.List[i].isRead == 0 && KT_Read_List.List[i].reg == 0 &&
                            KT_Read_List.List[i].value == 0)
                        {
                            KT_Read_List.List[i].reg = reg;
                            KT_Read_List.List[i].value = 0;
                            KT_Read_List.List[i].isRead = 1;
                            KT_Read_List.List[i].isReadDone = 0;
                            KT_Read_List.total++;
                            break;
                        }
                    }
                }
                goto NEXT_TOKEN;
            }

            /** Lệnh tắt màn hình LED sau N giây không sử dụng */
            /** Điều khiển local mode về IDLE ngay lập tức */
            if (strstr(mToken, "DISP_DELAY="))
            {
                uint16_t delay = GetNumberFromString(11, mToken);
                if (xSystem.Parameter.DispDelay != delay)
                {
                    xSystem.Parameter.DispDelay = delay;
                    configNum++;
                }
                goto NEXT_TOKEN;
            }
            /* Next */
        NEXT_TOKEN:
            mToken = strtok(NULL, ",");
        }

        /** Luu cau hinh */
        if (configNum)
        {
            if (xSystem.Parameter.enter_test_mode == 0)
            {
                InternalFlash_WriteConfig();
            }
        }
    }

    //	uint8_t Index = 0;
    //	uint8_t SoLanXuatHien = 0;
    //	uint16_t ViTriXuatHien[5] = {0};
    //	uint8_t MessageLength = 0;

    //	//Xac dinh vi tri cac ban tin
    //	for(Index = 0; Index < BlackBoxBuffer.BufferIndex; Index++)
    //	{
    //		if(BlackBoxBuffer.Buffer[Index] == '$' && BlackBoxBuffer.Buffer[Index + 1] == 'B' &&
    //			 BlackBoxBuffer.Buffer[Index + 2] == 'A' && BlackBoxBuffer.Buffer[Index + 3] == 'P' &&
    //			 BlackBoxBuffer.Buffer[Index + 4] == 'C')
    //		{
    //			ViTriXuatHien[SoLanXuatHien] = Index;
    //			SoLanXuatHien++;
    //			ViTriXuatHien[SoLanXuatHien] = 0;
    //		}
    //	}
    //
    //	if(SoLanXuatHien == 0) return;
    //
    //	//Xu ly ban tin Blackbox (tach ban tin trong truong hop bi dinh lien)
    //	for(Index = 0; Index < SoLanXuatHien; Index++)
    //	{
    //		if(ViTriXuatHien[Index + 1] > ViTriXuatHien[Index])
    //		{
    //			MessageLength = ViTriXuatHien[Index + 1] - ViTriXuatHien[Index];
    //			memcpy(BanTinBLB, &BlackBoxBuffer.Buffer[ViTriXuatHien[Index]], MessageLength);
    //		}
    //		else
    //		{
    //			MessageLength = BlackBoxBuffer.BufferIndex - ViTriXuatHien[Index];
    //			memcpy(BanTinBLB, &BlackBoxBuffer.Buffer[ViTriXuatHien[Index]], MessageLength);
    //		}
    //		XuLyBanTin(BanTinBLB, MessageLength);
    //		memset(BanTinBLB, 0, MessageLength);
    //	}
}
