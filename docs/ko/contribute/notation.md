# 용어

다음 용어, 기호, 문자는 이 안내서 전체의 텍스트와 다이어그램에 활용합니다.

## 표기법

- Bold face variables indicate vectors or matrices and non-bold face variables represent scalars.
- The default frame for each variable is the local frame: $\ell{}$.
  Right [superscripts](#superscripts) represent the coordinate frame.
  If no right superscript is present, then the default frame $\ell{}$ is assumed.
  An exception is given by Rotation Matrices, where the lower right subscripts indicates the current frame and the right superscripts the target frame.
- Variables and subscripts can share the same letter, but they always have different meaning.

## 약어

| 약어          | 설명                                                                                                                     |
| ----------- | ---------------------------------------------------------------------------------------------------------------------- |
| AOA         | Angle Of Attack(받음각). Also named _alpha_.                           |
| AOS         | Angle Of Sideslip(횡활각). Also named _beta_.                          |
| FRD         | 오른손 법칙에 따라 기체의 앞부분을 X축, 오른쪽 방향을 Y축, 아래 방향을 Z축으로 두는 좌표계                                                                 |
| FW          | Fixed-wing (planes).                                                                |
| MC          | MultiCopter(멀티콥터).                                                                  |
| MPC 또는 MCPC | MultiCopter Position Controller(멀티콥터 위치 조종기). MPC는 모델 예측 제어라고도 합니다. |
| NED         | 오른손 법칙에 따라  X 축은 진북을 가리키고 Y 축은 진동, Z 축은 아래를 가리키는 좌표계                                                                   |
| PID         | 비례, 적분 및 미분에 따른 콘트롤러.                                                                                  |

## 기호

| 변수                                                    | 설명                                                                                                                                                                                        |
| ----------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| $x,y,z$                                               | x, y, z 각 좌표를 따르는 변환                                                                                                                                                                      |
| $\boldsymbol{\mathrm{r}}$                             | $$\boldsymbol{\mathrm{r}} = [x \quad y \quad z]^{T}$$ 위치 벡터                                                                                                                           |
| $\boldsymbol{\mathrm{v}}$                             | $$\boldsymbol{\mathrm{v}} = \boldsymbol{\mathrm{\dot{r}}}$$ 속도 벡터                                                                                                                    |
| $\boldsymbol{\mathrm{a}}$                             | $$\boldsymbol{\mathrm{a}} = \boldsymbol{\mathrm{\dot{v}}} = \boldsymbol{\mathrm{\ddot{r}}}$$ 가속도 벡터                                                                               |
| $\alpha$                                              | Angle of Attack(공격 각도, AOA).                                                                                                                           |
| $b$                                                   | 주익 길이 (끝에서 끝까지)                                                                                                                                                        |
| $S$                                                   | 주익 넓이                                                                                                                                                                                     |
| $AR$                                                  | 가로 세로 비율: $AR = b^2/S$                                                                                                                                                    |
| $\beta$                                               | 횡활각(Angle of sideslip, AOS).                                                                                                                           |
| $c$                                                   | 주익현 길이                                                                                                                                                                                    |
| $\delta$                                              | 기체 역학 제어 표면 이탈각(손실각). 손실 값이 양인 경우 음수 값의 모멘트를 생성합니다.                                                                                    |
| $\phi,\theta,\psi$                                    | 오일러 각. roll(=Bank), pitch, yaw(=Heading).                                                                           |
| $\Psi$                                                | 자세 벡터: $\Psi = [\phi \quad \theta \quad \psi]^T$                                                                      |
| $X,Y,Z$                                               | x, y, z 축 방향의 힘                                                                                                                                                                           |
| $\boldsymbol{\mathrm{F}}$                             | $$\boldsymbol{\mathrm{F}}= [X \quad Y \quad Z]^T$$ 힘 벡터                                                                                                                               |
| $D$                                                   | 견인력                                                                                                                                                                                       |
| $C$                                                   | 측풍력                                                                                                                                                                                       |
| $L$                                                   | 양력                                                                                                                                                                                        |
| $g$                                                   | 중력                                                                                                                                                                                        |
| $l,m,n$                                               | x, y, z 좌표 축 모멘트                                                                                                                                                                          |
| $\boldsymbol{\mathrm{M}}$                             | $$\boldsymbol{\mathrm{M}} = [l \quad m \quad n]^T$$ 모멘트 벡터                                                                                                                            |
| $M$                                                   | 마하 계수. Can be neglected for scale aircraft.                                                                                                               |
| $\boldsymbol{\mathrm{q}}$                             | 4원수 벡터.                                                                                                                                                                   |
| $\boldsymbol{\mathrm{\tilde{q}}}$                     | Hamiltonian attitude quaternion (see `1` below)                                                                                                                        |
| $\boldsymbol{\mathrm{R}}_\ell^b$ | 회전 행렬. $$\ell$$ 프레임에서 $$b$$ 프레임으로의 벡터 회전. $\boldsymbol{\mathrm{v}}^b = \boldsymbol{\mathrm{R}}_\ell^b \boldsymbol{\mathrm{v}}^\ell$ |
| $\Lambda$                                             | 끝단 젖힘각                                                                                                                                                                                    |
| $\lambda$                                             | 테이퍼 비율: $\lambda = c_{tip}/c_{root}$                                                                                            |
| $w$                                                   | 풍속                                                                                                                                                                                        |
| $p,q,r$                                               | 몸통 축 x, y, z 주변의 각율(각가속도)                                                                                                                                              |
| $\boldsymbol{\omega}^b$                               | 기체 프레임의 각속도 벡터: $\boldsymbol{\omega}^b = [p \quad q \quad r]^T$                                                       |
| $\boldsymbol{\mathrm{x}}$                             | 일반 상태 벡터                                                                                                                                                                                  |

- `1` Hamiltonian attitude quaternion. $$\boldsymbol{\mathrm{\tilde{q}}} = (q_0, q_1, q_2, q_3) = (q_0, \boldsymbol{\mathrm{q}})$$.<br> $$\boldsymbol{\mathrm{\tilde{q}}}$$ 로컬 프레임 $$\ell$$에 상대적인 고도를 설명합니다. To represent a vector in local frame given a vector in body frame, the following operation can be used: $\boldsymbol{\mathrm{\tilde{v}}}^\ell = \boldsymbol{\mathrm{\tilde{q}}} \, \boldsymbol{\mathrm{\tilde{v}}}^b \, \boldsymbol{\mathrm{\tilde{q}}}^_{}$ (or $\boldsymbol{\mathrm{\tilde{q}}}^{-1}{}$ instead of $\boldsymbol{\mathrm{\tilde{q}}}^_{}$ if $\boldsymbol{\mathrm{\tilde{q}}}{}$ is not unitary). $\boldsymbol{\mathrm{\tilde{v}}}{}$ represents a _quaternionized_ vector: $\boldsymbol{\mathrm{\tilde{v}}} = (0,\boldsymbol{\mathrm{v}})$

### 아래첨자 / 인덱스

| 아래첨자 / 인덱스 | 설명                                                |
| ---------- | ------------------------------------------------- |
| $a$        | 보조익(Aileron).  |
| $e$        | 승강타(Elevator). |
| $r$        | 방향타(Rudder).   |
| $Aero$     | 비행 역학                                             |
| $T$        | 추진력                                               |
| $w$        | 상대 항속                                             |
| $x,y,z$    | x, y, z 축 벡터 요소                                   |
| $N,E,D$    | 북, 동, 하 글로벌 방위에 따른 벡터 요소                          |

<a id="superscripts"></a>

### 위첨자 / 인덱스

| 위첨자 / 인덱스 | 설명               |
| --------- | ---------------- |
| $\ell$    | 로컬 프레임 PX4 기본 변수 |
| $b$       | 바디 프레임           |
| $w$       | 윈드 프레임           |

## 장식 기호

| 장식 기호                           | 설명     |
| ------------------------------- | ------ |
| $()^\*$      | 켤레 복소수 |
| $\dot{()}$   | 시간 미분  |
| $\hat{()}$   | 추정     |
| $\bar{()}$   | 평균     |
| $()^{-1}$    | 역행렬    |
| $()^T$       | 전치행렬   |
| $\tilde{()}$ | 쿼터니온   |
