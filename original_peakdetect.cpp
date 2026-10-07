void peakDetect(){
    int iCh;
    
    for ( iCh = 0; iCh < NUMCHANS; iCh++ ){
        if ( isRAILED_Ch[iCh] ) iCh_Trg_rail = iCh;
        if ( iCh == NUMCHANS-1 && !isRAILED_Ch[iCh_Trg_rail] ) w_rail = 0;

        if ( w == 0 && Signal[iCh].value <= Signal[iCh].bkg + THRESHOLD_START[iCh] && !TRIGGERED && !isRAILED_Ch[iCh] ) STATE = QUIET;
        if ( w == 0 && Signal[iCh].value >  Signal[iCh].bkg + THRESHOLD_START[iCh] && !TRIGGERED && !isRAILED_Ch[iCh] ) {
            STATE = PEAK; TRIGGERED = true ;
            iCh_Trg = iCh;
            Signal[iCh].start = Signal[iCh].value;

            if( Signal[iCh].start - Signal[iCh].bkg > THRESHOLD_START[iCh]*5 ) {STATE = SHORT; TRIGGERED = false;}
        }
        
        if ( iCh == iCh_Trg ){
            if ( w >  ( iCh == FWD_CH ? WIDTH_MAX/7 : WIDTH_MAX )						   		     	  &&  TRIGGERED ) { STATE = LONG;  TRIGGERED = false; }
            if ( w >= AR_WIDTH_MIN && (float) ( Signal[iCh].max   - Signal[iCh].start )/w < AR_MIN	      &&  TRIGGERED ) { STATE = SHORT; TRIGGERED = false; }
            if ( w >  WIDTH_MIN    &&	  	  ( Signal[iCh].value < Signal[iCh].bkg + THRESHOLD_END[iCh]) &&  TRIGGERED ) {
                if ( isPeakValid(iCh) ) {
                    STATE = END;
                }else{
                    STATE = QUIET; TRIGGERED = false;
                }
            }
        }
    }
    
    if ( isRAILED_Ch[0] || isRAILED_Ch[1] || isRAILED_Ch[2] || 
         isRAILED_Ch[3] || isRAILED_Ch[4] || isRAILED_Ch[5] ){
		if( !tx_isRAILED ){
			w_rail++; 
            if( w_rail == RAILED_WIDTH ){
	                prevSTATE = STATE; STATE = RAILED;
	                prev_w = w; w = -1;
	                tx_isRAILED = true;
            }
		}
	}
    
    for ( iCh = 0; iCh < NUMCHANS; iCh++ ){
        switch( STATE ){
            case SHORT:
				Signal[iCh].bkg = Signal[iCh].bkg + (float) ( Signal[iCh].value - Signal[iCh].bkg )/(NORMALIZATION/ADJUST);
				re_initialize(iCh);
                break;
            case LONG:
				Signal[iCh].bkg = Signal[iCh].value;
				re_initialize(iCh);
                break;
            case QUIET:
				Signal[iCh].bkg += (float) (Signal[iCh].value - Signal[iCh].bkg) / NORMALIZATION;
				re_initialize(iCh);
                break;
            case PEAK:
				Signal[iCh].sum += Signal[iCh].value;
				if ( Signal[iCh].value > Signal[iCh].max ){
					 Signal[iCh].max   = Signal[iCh].value;
					 Signal[iCh].t_max = w;
				}
				if( iCh == iCh_Trg ) THRESHOLD_END[iCh] = ( Signal[iCh].max - Signal[iCh].bkg ) * THRESHOLD_END_PCT[iCh] + THRESHOLD_START[iCh];
				if( iCh == NUMCHANS-1 ) w++;
                break;
            case END:
                push_event(iCh);
                re_initialize(iCh);
                if ( iCh == NUMCHANS-1 ) {
                    TRIGGERED = false;
                    event_transmission_ok();
                }
                break;
            case RAILED:
				if( tx_isRAILED ){
                    push_event(iCh);
                    if( isRAILED_Ch[iCh] )Signal[iCh].max = Signal[iCh].bkg = Signal[iCh].value;
                    if ( iCh == NUMCHANS-1 ) {
                        event_transmission_ok();
                        w_rail = -tx_isRAILED_Rate/2; tx_isRAILED = false;
						STATE = prevSTATE; w = prev_w;
                    }
                }
                break;
		}
	}
    
}