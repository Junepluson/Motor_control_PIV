//=============================================================================================
// File name : DC_PIV_Con_HSJ.ino
//---------------------------------------------------------------------------------------------
// <개요 (Abstract)>
// 이 프로젝트는 Arduino Due 보드를 기반으로,
// DC 모터의 위치-속도 제어를 위한 PIV 제어기(Position–Integral–Velocity Controller)를 구현합니다.
//
//   • DWT (Data Watchpoint and Trace) 사이클 카운터: 마이크로초 단위의 정밀한 시간 측정
//   • TC0 쿼드러처 인코더(QDEC): 모터 엔코더의 위치 및 속도 추정
//   • PIO 인터럽트: 외부 PWM 신호를 측정하여 목표 위치(ref_theta)로 변환
//   • PWM 출력: H-브리지를 통해 제어 전압 생성 (중앙정렬 PWM 사용)
//   • Serial 통신: 제어 결과 실시간 디버깅 출력
//
// 이 구조를 통해 실시간 제어 루프 내에서 정확한 5ms 샘플링 주기를 유지하며,
// 목표 위치와 실제 위치 간의 오차를 기반으로 DC 모터를 정밀하게 제어할 수 있습니다.
//---------------------------------------------------------------------------------------------
// <하드웨어 핀 연결 (Pin Mapping)>
//
//   • Encoder A   → PIOB 25 (PB25, TC0 Channel0 TIOA0)    : 엔코더 A상 입력 (Quadrature decoding)
//   • Encoder B   → PIOB 27 (PB27, TC0 Channel0 TIOB0)    : 엔코더 B상 입력
//   • PWM 출력     → PIOC 2  (PC2, PWMH0)                 : H-브릿지에 전달되는 PWM 신호
//   • Direction    → PIOC 3  (PC3)                       : 모터 회전 방향 제어 (GPIO 출력)
//   • RC 입력      → PIOC 5  (PC5)                       : 외부 PWM 입력 인터럽트 (duty_ratio 측정용)
//   • Serial TX    → D1 (TX0)                            : PC <-> 아두이노 간 디버깅용 시리얼 통신 (115200 bps)
//---------------------------------------------------------------------------------------------
//
// 1) setup()
//   - configure_encoder_counter() : 엔코더 카운터를 TC0의 쿼드러처 모드로 설정
//   - configure_PWM()             : PWM 출력 채널 설정 (dead time 포함)
//   - configure_PIO()             : 외부 PWM 입력 인터럽트 설정
//   - DWT_Init()                  : DWT 사이클 카운터 활성화
//   - 샘플링 주기 계산 및 초기화
//
// 2) loop() [제어 루프: 5ms 주기 유지]
//   - readEncoderAndUpdate()   : 현재 엔코더 카운트 → 위치/속도 계산
//   - computeTargetAngle()     : 입력 duty_ratio 기반 목표 위치(ref_theta) 계산
//   - computeAndSetPWM()       : PIV 제어기 연산 → 방향 설정 + PWM 출력 + 디버깅 출력
//   - maintainSamplingPeriod() : DWT를 활용한 정확한 루프 타이밍 유지
//
// 3) 인터럽트 처리
//   - PIOC_Handler(): 외부 PWM 신호의 상승/하강 엣지를 감지하여 duty_ratio 계산
//   - updateDutyRatio(): DWT_CYCCNT를 기반으로 High 시간 계산 → 듀티비 환산 (%)
//
//---------------------------------------------------------------------------------------------
// <프로젝트 목적>
//   • 실시간 임베디드 시스템 상에서 PIV 제어기의 구현과 튜닝 방법 학습
//   • 고해상도 시간 측정(DWT)과 인터럽트 기반 센싱의 통합
//   • 모터 제어 루프의 시간 동기화와 안정성 확보
//
//---------------------------------------------------------------------------------------------
// Date   : 2025-06-08
// Author : 홍성준
// Revision : v21.0
//---------------------------------------------------------------------------------------------
// LINK : https://youtube.com/shorts/FDpVmk2tysQ?si=L2GrEkVG0RMvbFZ8
//=============================================================================================


#define DWT_CTRL (*((volatile uint32_t*)(0xE0001000UL)))   // DWT 제어 레지스터: CYCCNT(사이클 카운터) 사용 가능화 비트 설정 용도
#define DWT_CYCCNT (*((volatile uint32_t*)(0xE0001004UL))) // DWT 사이클 카운터 레지스터: 활성화 시 CPU 클럭 사이클 수를 누적 카운트
#define PI 3.141592653589793                               // 원주율
#define PPR 64                                             //Encoder의 Pulse Per Revolution

// 하이퍼 파라미터 gain 값 설정
#define P_gain 35     // 위치 오차에 대한 비례 이득 (Kp) 38 35
#define I_gain 5      // 속도 오차에 대한 적분 이득 (Ki) 4
#define V_gain 0.077  // 속도(feedback) 이득 (Kv) 0.08
#define A_gain 100    // 가속도 변화량(anti-windup) 보상(Ka) 

//PIO
void configure_PIO();

//PWM
void configure_PWM();
volatile double duty_ratio;       // PWM 듀티 사이클 비율 (0.0 ~ 1.0): PWM 채널의 CDTY / CPRD 값으로 결정

//Timer/Counter 
void DWT_Init();                  // DWT를 사용할 수 있도록 초기화하는 함수 
float sample_time = 0.005;        // sampling time. 이 부분을 다양하게 바꿔 시도해 볼 것
uint32_t MicrosSampleTime;        // micro-second 단위로 환산한 sample time
uint32_t MicrosCycle;
uint32_t SampleTimeCycle;         // sample time에 해당하는 instrution (또는 clock) cycle 수
uint32_t start_time, end_time;    // 주기적 연산의 시작시간과 종료시간. instruction cycle의 수를 단위로 사용
int start_point, end_point;       // 데이터 처리 버퍼 시작과 종료 인덱스 지정용 변수

//Encoder Counter
void configure_encoder_counter();
double w_rad;            // 모터 각속도(rad/s): (N(t) - N(t-Δt)) / Δt로 계산된 속도 값
int32_t cnt1;            // 현재 엔코더 누적 카운트 값
int32_t cnt1_p;          // 이전 사이클 엔코더 카운트 값 저장용 (속도 계산 시 차분 비교)
float count_to_radian;   // 엔코더 카운트를 라디안 단위로 변환하기 위한 계수 (2π / PPR)
double ref_theta;        // 목표 위치 각도(rad): 제어기로부터 입력받은 명령 각도 저장
double ref_theta_p = 0;  // 이전 목표 위치 각도 저장 (차분 계산용)
double theta;            // 현재 측정된 위치 각도(rad): 엔코더 카운트를 라디안으로 변환한 값
 
class PIV {
public:
  // 제어기 파라미터
  double Kp, Ki, Kv, Ka;             // Kp: 위치 오차 비례 이득, Ki: 속도 오차 적분 이득
                                     // Kv: 속도 피드백 이득, Ka: anti-windup 보상 이득
  double ts;                         // 제어기 샘플링 주기 (초 단위)

  // 내부 상태 변수
  double w_rad;                      // 현재 각속도 (rad/s), 엔코더 기반 실시간 속도
  double integral;                   // 적분기 누적값 (속도 오차 적분)
  double anti_w_err;                 // anti-windup 보상항 (포화된 출력과 계산 출력 차이)

  // 출력 제한값 (±12V 제한)
  const double output_max = 12.0;    // 제어 출력 최대값 (상한)
  const double output_min = -12.0;   // 제어 출력 최소값 (하한)

  // 엔코더 보정용 변수
  double count_to_radian;            // 엔코더 카운트를 라디안으로 변환하는 계수 (2π / PPR)
  int32_t cnt_p = 0;                 // 이전 루프의 엔코더 카운트 (속도 계산용)

  // 생성자
  PIV(double kp, double ki, double kv, double ka, double sample_time, double c2r)
    : Kp(kp), Ki(ki), Kv(kv), Ka(ka), ts(sample_time), count_to_radian(c2r) {}  // 초기값 설정

  // 외부 PWM 듀티비 기반 목표 위치(ref_theta) 계산
  void updateDutyRatio(double duty_ratio) {
    ref_theta = 200.0 / 60.0 * duty_ratio - 500.0 / 3.0;      // 입력 듀티비(%)를 위치(rad)로 환산
  }

  // 현재 엔코더 카운트(cnt) 기반 위치와 속도 계산
  void updateEncoder(int32_t cnt) {
    theta = count_to_radian * cnt;                            // 현재 위치(rad) = 카운트 × 변환계수
    w_rad = count_to_radian * (cnt - cnt_p) / ts;             // 속도(rad/s) = 위치 변화량 / 시간
    cnt_p = cnt;                                              // 현재 카운트를 다음 루프의 기준으로 저장
  }

  // PIV 연산 수행 후 출력 전압 반환
  double control() {

    // 목표 위치의 변화율 및 현재 위치 오차 계산
    double r_dot     = (ref_theta - ref_theta_p) / ts;         // ref_theta의 시간당 변화량 (rad/s) 계산 → 목표 속도 성분
    double err_theta = ref_theta - theta;                      // 현재 위치 오차 (rad) = 목표 위치 - 현재 위치
    double w_ref     = Kp * err_theta + r_dot;                 // 목표 속도 계산: 위치 오차에 비례한 속도 + 목표 위치 변화율
    double w_err     = w_ref - w_rad;                          // 속도 오차 (rad/s) = 목표 속도 - 실제 측정 속도


    //  제어기 내부 상태 업데이트 및 출력 계산
    integral += ts * (Ki * w_err + Ka * anti_w_err);           // 적분항 누적: 속도 오차에 대한 적분 + anti-windup 보정항 포함
                                                              // → 제어기의 장기 누적 오차를 보정하며, 포화로 인한 왜곡도 방지                                                  
    double out_tmp = Kv * w_err + integral;                    // 제어 출력(가상 전압) 계산: 속도 오차에 대한 비례 출력 + 누적 보정값


    //  출력 포화 처리 (전압 범위 제한)
    double out;
    if      (out_tmp > output_max) out = output_max;           // 출력이 허용 최대값을 넘으면 최대값으로 제한
    else if (out_tmp < output_min) out = output_min;           // 출력이 최소값보다 작으면 최소값으로 제한
    else                          out = out_tmp;               // 포화 범위 내라면 그대로 사용


    //  anti-windup 보정 및 상태 업데이트
    anti_w_err  = out - out_tmp;                               // 포화 차이 계산: 실제 출력과 내부 출력값의 차이 → 누적 오차 보정용
                                                              // → 포화로 인해 제어기가 "더 세게 동작하고 싶었지만 못한" 차이 반영
    ref_theta_p = ref_theta;                                   // 현재 목표 위치 저장 → 다음 루프에서 r_dot 계산에 사용
    return out;                                                // 최종 제어 전압 출력 반환 (±12V 범위)
  }
};

void setup() {
  Serial.begin(115200);          // serial 통신 초기화
  configure_PWM();               // PWM을 사용하기 위한 설정수행
  configure_PIO();               // PIO을 사용하기 위한 설정수행
  configure_encoder_counter();   // Encoder Counter를 사용하기 위한 설정수행

  MicrosSampleTime = (uint32_t)(sample_time * 1e6);     // sample time에 해당하는 micro second 단위의 정수를 계산
  MicrosCycle = SystemCoreClock / 1000000;              // Due의 SystemCoreClock은 84MHz
  SampleTimeCycle = MicrosCycle * MicrosSampleTime;
  // 84 × 5000 = 420,000 사이클  
  // 이만큼 사이클이 지나면 한 번의 제어 주기가 끝난 것으로 본다

  DWT_Init(); //DWT_CTRL, DWT_CYCCNT 레지스터 설정, DWT를 intialize 시킨다.
  
  //DWT 사이클 카운터 방식
  start_time = DWT_CYCCNT;                 // 시작 시점 카운터 저장 
  end_time = start_time + SampleTimeCycle; // 현재 사이클 값에 (420 000)를 더해, 이 사이클 수가 되면 제어 함수를 다시 실행

  cnt1_p = 0;                              // 속도 계산 시 “이전 카운트”로 활용 
  count_to_radian = 2 * PI / PPR;          // 한 펄스당 2π/PPR(rad) 만큼 회전 (PPR=64)
  Serial.print("PWM_CMR = 0x");
  Serial.println(PWM->PWM_CH_NUM[0].PWM_CMR, HEX);
}

PIV piv(P_gain, I_gain, V_gain, A_gain, sample_time, 2 * PI / PPR);  // PIV 제어기 객체 생성

// 주기적으로 모터의 위치와 속도를 읽고, 목표 위치와 비교하여 PIV 제어 연산을 수행한 뒤,
// 방향과 PWM 신호를 출력합니다. 샘플링 주기는 DWT 사이클 카운터를 이용해 5ms로 고정 유지됩니다.
void loop() {
  readEncoderAndUpdate();        // 현재 엔코더 값을 기반으로 위치/속도 계산
  computeTargetAngle();          // 외부에서 입력된 PWM duty 비율을 기준으로 목표 위치(ref_theta) 설정
  computeAndSetPWM();            // 제어기 계산 → 모터 방향 설정 → PWM 듀티 업데이트 → 디버깅 출력
  maintainSamplingPeriod();      // DWT 사이클 기반 정확한 샘플링 주기(5ms) 유지
}

// [엔코더 입력 처리]
// TC0 타이머의 현재 카운터 값을 읽어 엔코더의 위치를 확인하고,
// 이전 카운트와의 차이를 통해 각속도(w_rad)를 계산하여 PIV 제어기 상태를 갱신합니다.
void readEncoderAndUpdate() {
  int32_t cnt = TC0->TC_CHANNEL[0].TC_CV;        // 엔코더 현재 카운트 (position count)
  piv.updateEncoder(cnt);                        // 카운트를 라디안으로 변환해 theta, w_rad 업데이트
}

// [목표 위치 계산]
// 외부에서 측정된 PWM 듀티 비율(duty_ratio)을 기반으로 목표 위치(ref_theta)를 계산합니다.
// 듀티 0~100% → 위치 -500/3 ~ +500/3 rad로 선형 매핑
void computeTargetAngle() {
  piv.updateDutyRatio(duty_ratio);               // 목표 각도(ref_theta) 갱신
}


// [PIV 제어 및 모터 구동 출력]
// 1. 위치 오차를 기반으로 목표 속도 계산
// 2. 속도 오차에 대해 PIV 연산 수행 → 제어 전압 도출
// 3. 제어 전압 부호에 따라 방향 핀 설정
// 4. PWM duty 계산 및 모터 출력
// 5. Serial 디버깅 출력
void computeAndSetPWM() {
  double control_voltage = piv.control();        // PIV 제어기 연산 수행 (출력: ±12V)
  double duty = 100.0 / 12.0 * control_voltage;  // 전압(±12V)을 퍼센트 duty(0~100%)로 선형 변환

  setMotorDirection(duty);                       // 전압 부호에 따라 CW/CCW 방향 설정
  updatePWMOutput(duty);                         // PWM 채널에 duty 설정값 갱신
  debugSerialPrint();                            // Serial 출력으로 ref_theta와 현재 위치 확인
}

// [모터 방향 설정]
// 제어 전압이 음수일 경우 역방향(CCW), 양수일 경우 정방향(CW)으로 모터 회전
// 아두이노 Due의 PIOC 3번 핀(GPIO 출력 핀)을 사용하여 H-브리지 방향을 제어
void setMotorDirection(double &duty) {
  if (duty < 0) {
    duty = -duty;                                // PWM duty는 항상 양수로 입력해야 하므로 절댓값 처리
    PIOC->PIO_ODSR &= ~PIO_PC3;                  // 방향 핀 LOW → 반시계 방향(CCW)
  } else {
    PIOC->PIO_ODSR |= PIO_PC3;                   // 방향 핀 HIGH → 시계 방향(CW)
  }
}

// [PWM 출력 설정]
// 계산된 duty 값을 기반으로 PWM 채널의 듀티 설정 레지스터(CDTYUPD)를 갱신합니다.
// 중앙 정렬 방식 보정을 반영한 duty 계산식 사용
void updatePWMOutput(double duty) {
  uint32_t pwm_value = (duty / 100.0) * 2100;
  PWM->PWM_CH_NUM[0].PWM_CDTYUPD = pwm_value;  // 듀티 설정값 업데이트
  PWM->PWM_SCUC = 1;                           // PWM 설정 즉시 반영 (software trigger)
}

// [디버깅용 출력]
// Serial 모니터에 목표 위치(ref_theta)와 실제 위치(theta)를 출력하여 제어 성능 확인
void debugSerialPrint() {
  Serial.print(ref_theta);                     // 목표 각도 출력
  Serial.print(" ");
  Serial.println(theta);                       // 현재 측정된 각도 출력
}

// [샘플링 주기 유지]
// DWT 사이클 카운터(DWT_CYCCNT)를 활용해 정확히 5ms마다 loop 반복되도록 동기화
void maintainSamplingPeriod() {
  while (!((end_time - DWT_CYCCNT) & 0x80000000));    // 5ms 동안 대기 (unsigned 오버플로우 활용)
  end_time += SampleTimeCycle;                        // 다음 제어 루프 종료 시점 예약
}

//----------------------------------------------------------------------
// Arduino Due의 모든 PIO는 external interrupt를 설정할 수 있다.
//---------------------------------------------------------------------- 
void configure_PIO() {
  pmc_enable_periph_clk(ID_PIOC);         // PIOC의 clock을 공급한다.  
  PIOC->PIO_PER |= PIO_PC3 | PIO_PC5;     // PC3, PC5 PIO 기능 enable
  PIOC->PIO_IDR |= PIO_PC3 | PIO_PC5;     //Interrupt Disable Register

  PIOC->PIO_OER |= PIO_PC3;               // output enable
  PIOC->PIO_OWER |= PIO_PC3;              // output write enable
  PIOC->PIO_CODR |= PIO_PC3;              //clear output data

  PIOC->PIO_ODR |= PIO_PC5;               // PC5를 input으로 설정
  PIOC->PIO_IFER |= PIO_PC5;              // input glitch filter enable  
  PIOC->PIO_IER |= PIO_PC5;               // interrupt enable
  
  PIOC->PIO_DIFSR |= PIO_PC5;             // 디바운스 필터 설정 (느린 글리치 제거)
  PIOC->PIO_SCIFSR |= PIO_PC5;            // 시스템‐클럭 기반 글리치 필터 설정 선택 (초고속 스파이크 제거) 없으면 동작 어려움

  NVIC_DisableIRQ(PIOC_IRQn);             // 일단 Interrupt disable
  NVIC_ClearPendingIRQ(PIOC_IRQn);        // pending clear
  NVIC_SetPriority(PIOC_IRQn, 1);         // SysTick(0)보다 한 단계 낮은 우선순위
  NVIC_EnableIRQ(PIOC_IRQn);              // Interrupt enable
}

void configure_PWM() {
  //----------------------------------------------------------------------  
    // PWM을 발생하기 위해서는 해당 Pin을 PIO가 아니라  
    // Peripheral function 기능을 사용하여야 한다.  
    // Pin의 peripheral function을 활성화 시키기 위해서는  
    // 해당 pin의 PIO, 즉 GPIO로 사용하는 기능을 disable  
    // 시켜야 한다. doc11057의 page 656 참조  
  //----------------------------------------------------------------------
  PIOC->PIO_PDR = PIO_PC2;                // PC2의 PIO를 disable 시킨다.
  //----------------------------------------------------------------------  
    // PC2를 PWM 출력 pin으로 설정하자. Pin은 PIO가 아니라  
    // peripheral function 기능으로 사용되어야 한다. PC2를  
    // Peripheral B로 설정해야 PWM 출력 pin으로 사용할 수 있다. 
    //PIO_ABSR의 해당 bit에 0을 쓰면 Peripheral A function,  
    // 1을 쓰면 Peripheral B function으로 설정된다.  
    //----------------------------------------------------------------------  

  PIOC->PIO_ABSR |= PIO_PC2;              // PC2를 Peripheral B function 설정한다  
  pmc_enable_periph_clk(ID_PWM);          // Peripheral PWM에 공급되는 clock을 enable 시킨다. 
  PWM->PWM_DIS = (1U << 0);               // PWM channel 0번 disable  

    /* --------------------------------------------------------------------------
       PWM Clock Generator 및 채널 설정 요약 (레지스터: PWM_CLK, PWM_CMR 등)
       --------------------------------------------------------------------------

       [1] 클럭 소스 및 분주 설정 (PWM->PWM_CLK)
         - PREA = 0b0000 : CLKA에 MCK(84 MHz) 직접 연결
         - PREB = 0b0000 : CLKB는 비활성화
         - DIVA = 1      : CLKA = MCK / 1 = 84 MHz
         - DIVB = 0      : CLKB 미사용

       [2] PWM 채널 모드 설정 (PWM_CMR 레지스터, 채널별)
         - CPRE  : 클럭 소스 선택
                   ▸ 0b1011 (11) → CLKA 사용
         - CALG  : 정렬 모드
                   ▸ 0 → Left-aligned (기본)
                   ▸ 1 → Center-aligned (주기 2배)
         - CPOL  : 출력 극성
                   ▸ 0 → Normal polarity (SET → CLEAR)
                   ▸ 1 → Inverted polarity (CLEAR → SET)
         - DTE   : Dead-time generator 활성화
         - DTH   : Dead-time High (하위 12bit 유효)
         - DTL   : Dead-time Low  (하위 12bit 유효)

       [3] PWM 주기 및 듀티 설정
         - CPRD  : PWM 한 주기의 카운트 수 (예: 2100)
         - CDTY  : PWM 듀티에 해당하는 카운트 수

       [4] 동기 업데이트 설정 (PWM_SCM)
         - SYNCx : 동기화할 채널 선택 (SYNC0, SYNC1, SYNC2)
         - UPDM  : 듀티 갱신 방식
                   ▸ 0 = Manual write (수동 업데이트)

       [5] PWM 출력 활성화 (PWM_ENA)
         - CHIDx 비트에 1을 쓰면 해당 채널의 PWM 출력이 enable됨

       ※ 참고 문서: SAM3X8E 데이터시트 (doc11057)
         - PWM_CLK: p.989, p.1020
         - PWM_CMR: p.1058
    */

  PWM->PWM_CLK &= ~0x0F000F00; // PREA=0, PREB=0, MCK를 사용하겠다는 말.
  PWM->PWM_CLK &= ~0x00FF0000; // DIVB=0, CLKB는 turned off 된다.
  PWM->PWM_CLK &= ~0x000000FF; // Clear DIVA
  PWM->PWM_CLK |= (1u << 0);   // DIVA = 1

  // Clock 선택 (CPRE 비트): MCK 분주(clock generator의 CLKA/CLKB) 또는 내부 클럭 중 하나를 PWM 채널에 할당  
  // 정렬 모드 (CALG 비트): 
  //   0 → Left-aligned (기본)  
  //   1 → Center-aligned (주기가 2배 느려짐)  
  // 극성 설정 (CPOL 비트): 
  //   0 → 카운터가 duty 값에 도달하면 출력 SET → 나머지 구간 CLEAR  
  //   1 → duty 값까지는 CLEAR → 그 이후 SET  
  // Dead-time 생성기 (DTE 비트): 채널별로 Dead-time 기능 활성화 → High/Low 전환 시 스파이크 방지  
  // High/Low 출력 매핑:  
  //   PWMxL과 PWMxH 핀 연결 방식에 따라 duty 비율 전달 방식 결정  
  //   필요 시 소프트웨어로 duty 반전 가능  
  PWM->PWM_CH_NUM[0].PWM_CMR &= ~0x0000000F;    // CPRE를 모두 0으로 clear 
  PWM->PWM_CH_NUM[0].PWM_CMR |= 0x0B;          // CPRE=0xB = 0b1011 (CLKA 선택)
  PWM->PWM_CH_NUM[0].PWM_CMR |= 1u << 8;        // CALG=1, center aligned 
  PWM->PWM_CH_NUM[0].PWM_CMR &= ~(1u << 9);     // CPOL=0
  //----------------------------------------------------------------------  
  // PWM channel period 설정  
  // 만약에 PWM channel이 disable 된 상태라면 PWM_CPRD에 period 값을 쓴다.  
  // 그렇지 않고 enable 된 상태라면 PWM_CPRDUPD에 period 값을 쓴다.  
  // 위에서 모두 disable을 시켜놨으므로 PWM_CPRD에 쓰면 된다.  
  //----------------------------------------------------------------------
  PWM->PWM_CH_NUM[0].PWM_CPRD = 2100; 
  //----------------------------------------------------------------------  
  // PWM channel duty 설정  
  // 만약에 PWM channel이 disable 된 상태라면 CDTY에 duty 값을 쓴다.  
  // 그렇지 않고 enable 된 상태라면 CDTYUPD에 duty 값을 쓴다.  
  // 위에서 disable 시켜놨으므로 CDTY에 쓰기로 한다.  
  //  
  // [주] duty는 period 보다 작거나 같아야 한다.  
  //----------------------------------------------------------------------
  PWM->PWM_CH_NUM[0].PWM_CDTY = 0;
  //----------------------------------------------------------------------  
  // PWM_SCM 설정  
  //   0x00000007 = 0b111 → SYNC2=1, SYNC1=1, SYNC0=1  
  // UPDM 비트는 0으로 남겨두어 Manual write of double buffer,  
  // manual update of synchronous channels 모드로 설정  
  //----------------------------------------------------------------------
  PWM->PWM_SCM |= 0x00000007;                     // O7 = 0b111, SYNC2=1, SYNC1=1, SYNC0=1  
  PWM->PWM_SCM &= ~0x00030000;                    // UPDM = 0, Manual write of double buffer,  
                                                  // manual update of synchronous channels  
  //----------------------------------------------------------------------  
  // PWM_ENA는 하위 8 bit가 CHID0~CHID7을 구성한다. 여기서는  
  // 해당 bit에 0을 쓰면 아무것도 안 일어나고 해당 bit에 1을 쓰면  
  // 해당 channel의 PWM output이 enable 되게 된다.  
  //----------------------------------------------------------------------
  PWM->PWM_ENA = 1u << 0;
}

//----------------------------------------------------------------------
// DWT : Debug Watchpoint and Trace Unit. 이것을 일단 사용하려면 먼저
// CoreDebug의 DEMCR(Debug Exception and Monitor Control Register)에 있는
// Trace Enable bit를 1로 만들어야 한다.
// 그러다음 DWT의 CYCCNT를 0으로 clear 시키고 counter를 enable 시키면
// instruction cycles를 세어주는 작업을 한다.
//--------------------------------------------------------------------
void DWT_Init(){
  //------------------------------------------------------------------
  // CoreDebug는 다행히도 core_cm3.h에 정의되어 있다.
  // Trace Enable bit를 1로 만들어서 DWT를 사용할 수 있도록 변경한다.
  //------------------------------------------------------------------
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT_CYCCNT = 0;                                     //Reset the counter
  DWT_CTRL |= 1;                                       // Enable the counter
}

//----------------------------------------------------------------------
// Quadrature pulse decoding을 위한 초기 설정을 수행한다. 여기서 주의할 것은
// TC0, TC1, TC2에 각각 3 channel timer counter가 있는데 이것들은 TC0~TC2까지의 이름을
// 가진다. TC0에는 TC0, TC1, TC2, TC1에는 TC3, TC4, TC5 그리고 TC2에는 TC6, TC7, TC8의 이름을
// 붙인다. 따라서 TC2의 첫번째 channel은 TC6이 된다. 따라서 clock 공급할 때 PID를
// ID_TC2가 아니라 ID_TC6으로 해야 된다. 이거 잘못 알고 있으면 낭패본다.
//----------------------------------------------------------------------
void configure_encoder_counter(){
  //------------------------------------------------------------------
  // TC0를 이용한 encoder counter 초기 설정.
  //------------------------------------------------------------------
  // PB25와 PB27를 quadrature pulse를 처리할 수 있는 Pin으로 기능설정해야 한다.
  PIOB->PIO_PDR = PIO_PB25 | PIO_PB27;                   // PB25, PB27의 PIO 기능 disable
  PIOB->PIO_ABSR |= (PIO_PB25 | PIO_PB27);               // PB25, PB27을 Peripheral B 기능으로 설정

  pmc_enable_periph_clk(ID_TC0);                         // TC0의 clock 공급

  TC0->TC_CHANNEL[0].TC_CMR = 5;                         // XC0를 clock source로 설정, mode=capture
  // quadrature decoder 및 위치 측정 모드 활성화 (필터 사용, MCK 30 clock)
  TC0->TC_BMR = (1 << 9) | (1 << 8) | (1 << 12) | (1 << 19) | (30 << 20); // x4 decoding (4체배)
  // clock enable 및 counter reset (SWTRG=1)
  TC0->TC_CHANNEL[0].TC_CCR = 5;                         // CLKEN=1 | SWTRG=1
}


//----------------------------------------------------------------------  
// WInterrupts.c에서 이미 PIOD_Handler가 정의되어 있으므로  
// 이 함수로 덮어쓰려면 WInterrupts.c에서 아래 선언이 필요하다.  
//   void PIOD_Handler(void) __attribute__((weak));  
// 인터럽트 서비스 루틴: PIOC 포트에서 PC5 핀에 상승 또는 하강 엣지가 발생했을 때 호출됨
//---------------------------------------------------------------------- 
void PIOC_Handler(void) {                               // PIOC 포트 인터럽트 서비스 루틴
  uint32_t status = PIOC->PIO_ISR;                      // 인터럽트 발생 상태 확인 (ISR 레지스터 읽기)
  uint32_t PC_value = PIOC->PIO_PDSR;                   // 현재 핀의 입력 상태 읽기 (PDSR 레지스터)

  if (!(status & PIO_PC5)) return;                      // 인터럽트가 PC5에서 발생하지 않았다면 종료

  if (PC_value & PIO_PC5) {                             // PC5가 HIGH라면 → 상승 엣지 감지
    start_point = DWT_CYCCNT;                           // 현재 사이클 카운터 값 저장 (상승 엣지 시점)
  } else {                                              // PC5가 LOW라면 → 하강 엣지 감지
    end_point = DWT_CYCCNT;                             // 현재 사이클 카운터 값 저장 (하강 엣지 시점)
    updateDutyRatio();                                  // 듀티비 계산 함수 호출
  }
}

void updateDutyRatio() {                                // 듀티비 계산 함수
  const double pwm_cnt = 84000.0;                       // PWM 전체 주기 클럭 수 (84MHz 기준, 1ms 주기)

  uint32_t diff;                                        // High 상태 지속 시간 저장 변수
  if (end_point >= start_point) {                       // 오버플로우 없는 정상 흐름
    diff = end_point - start_point;                     // 상승~하강 엣지 간의 시간 차 계산
  } else {                                              // DWT_CYCCNT 오버플로우 발생한 경우
    diff = (0xFFFFFFFF - start_point + end_point + 1);  // 오버플로우 보정 계산
  }

  duty_ratio = (double)diff / pwm_cnt * 100.0;          // 듀티비(%) = High 시간 / 전체 주기 × 100
}