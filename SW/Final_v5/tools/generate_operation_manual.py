from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (
    KeepTogether,
    PageBreak,
    Paragraph,
    SimpleDocTemplate,
    Spacer,
    Table,
    TableStyle,
)


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "output" / "pdf" / "Final_v5_발사_운용지침.pdf"

FONT_REGULAR = Path(r"C:\Windows\Fonts\NotoSansKR-Regular.ttf")
FONT_BOLD = Path(r"C:\Windows\Fonts\NotoSansKR-Bold.ttf")

NAVY = colors.HexColor("#102A43")
BLUE = colors.HexColor("#1677A6")
CYAN = colors.HexColor("#00A6C7")
INK = colors.HexColor("#1D2733")
SLATE = colors.HexColor("#52606D")
LIGHT = colors.HexColor("#F3F6F8")
LINE = colors.HexColor("#D8E1E8")
GREEN = colors.HexColor("#26734D")
GREEN_BG = colors.HexColor("#EAF6EF")
AMBER = colors.HexColor("#9A6700")
AMBER_BG = colors.HexColor("#FFF6D8")
RED = colors.HexColor("#B42318")
RED_BG = colors.HexColor("#FDECEA")
WHITE = colors.white


def register_fonts():
    if not FONT_REGULAR.exists() or not FONT_BOLD.exists():
        raise FileNotFoundError("Noto Sans KR fonts are required")
    pdfmetrics.registerFont(TTFont("NotoKR", str(FONT_REGULAR)))
    pdfmetrics.registerFont(TTFont("NotoKR-Bold", str(FONT_BOLD)))


register_fonts()
base = getSampleStyleSheet()

TITLE = ParagraphStyle(
    "TitleKR",
    parent=base["Title"],
    fontName="NotoKR-Bold",
    fontSize=27,
    leading=34,
    textColor=NAVY,
    alignment=TA_LEFT,
    spaceAfter=5 * mm,
)
SUBTITLE = ParagraphStyle(
    "SubtitleKR",
    parent=base["Normal"],
    fontName="NotoKR",
    fontSize=11,
    leading=17,
    textColor=SLATE,
    spaceAfter=4 * mm,
)
H1 = ParagraphStyle(
    "H1KR",
    parent=base["Heading1"],
    fontName="NotoKR-Bold",
    fontSize=18,
    leading=24,
    textColor=NAVY,
    spaceBefore=2 * mm,
    spaceAfter=4 * mm,
)
H2 = ParagraphStyle(
    "H2KR",
    parent=base["Heading2"],
    fontName="NotoKR-Bold",
    fontSize=12.5,
    leading=18,
    textColor=BLUE,
    spaceBefore=3 * mm,
    spaceAfter=2 * mm,
)
BODY = ParagraphStyle(
    "BodyKR",
    parent=base["BodyText"],
    fontName="NotoKR",
    fontSize=9.2,
    leading=14.2,
    textColor=INK,
    spaceAfter=2 * mm,
)
BODY_TIGHT = ParagraphStyle(
    "BodyTightKR",
    parent=BODY,
    fontSize=8.5,
    leading=12.5,
    spaceAfter=0,
)
SMALL = ParagraphStyle(
    "SmallKR",
    parent=BODY,
    fontSize=7.5,
    leading=10.5,
    textColor=SLATE,
    spaceAfter=0,
)
TABLE_HEAD = ParagraphStyle(
    "TableHeadKR",
    parent=BODY_TIGHT,
    fontName="NotoKR-Bold",
    textColor=WHITE,
    alignment=TA_CENTER,
)
TABLE_CELL = ParagraphStyle(
    "TableCellKR",
    parent=BODY_TIGHT,
    fontSize=8.1,
    leading=11.5,
)
TABLE_CELL_CENTER = ParagraphStyle(
    "TableCellCenterKR",
    parent=TABLE_CELL,
    alignment=TA_CENTER,
)
COMMAND = ParagraphStyle(
    "CommandKR",
    parent=BODY,
    fontName="NotoKR-Bold",
    fontSize=12,
    leading=16,
    textColor=NAVY,
    alignment=TA_CENTER,
    spaceAfter=0,
)
CALLOUT = ParagraphStyle(
    "CalloutKR",
    parent=BODY,
    fontName="NotoKR-Bold",
    fontSize=10,
    leading=15,
    alignment=TA_LEFT,
    spaceAfter=0,
)
FOOT = ParagraphStyle(
    "FootKR",
    parent=SMALL,
    fontSize=6.8,
    leading=9,
)


def para(text, style=BODY):
    return Paragraph(text, style)


def callout(title, body, kind="info"):
    palette = {
        "info": (BLUE, colors.HexColor("#EAF4F8")),
        "good": (GREEN, GREEN_BG),
        "warn": (AMBER, AMBER_BG),
        "danger": (RED, RED_BG),
    }
    accent, bg = palette[kind]
    content = para(
        f'<font color="{accent.hexval()}"><b>{title}</b></font><br/>{body}',
        CALLOUT,
    )
    table = Table([[content]], colWidths=[176 * mm])
    table.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, -1), bg),
                ("BOX", (0, 0), (-1, -1), 0.8, accent),
                ("LINEBEFORE", (0, 0), (0, -1), 4, accent),
                ("LEFTPADDING", (0, 0), (-1, -1), 12),
                ("RIGHTPADDING", (0, 0), (-1, -1), 10),
                ("TOPPADDING", (0, 0), (-1, -1), 9),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 9),
            ]
        )
    )
    return table


def data_table(rows, widths, header=True, font_size=8.1):
    cooked = []
    for row_index, row in enumerate(rows):
        cells = []
        for value in row:
            style = TABLE_HEAD if header and row_index == 0 else TABLE_CELL
            if style is TABLE_CELL and font_size != 8.1:
                style = ParagraphStyle(
                    f"cell-{font_size}-{row_index}",
                    parent=TABLE_CELL,
                    fontSize=font_size,
                    leading=font_size + 3,
                )
            cells.append(para(str(value), style))
        cooked.append(cells)

    table = Table(cooked, colWidths=widths, repeatRows=1 if header else 0)
    commands = [
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("GRID", (0, 0), (-1, -1), 0.45, LINE),
        ("LEFTPADDING", (0, 0), (-1, -1), 6),
        ("RIGHTPADDING", (0, 0), (-1, -1), 6),
        ("TOPPADDING", (0, 0), (-1, -1), 3),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
        ("ROWBACKGROUNDS", (0, 1 if header else 0), (-1, -1), [WHITE, LIGHT]),
    ]
    if header:
        commands.append(("BACKGROUND", (0, 0), (-1, 0), NAVY))
    table.setStyle(TableStyle(commands))
    return table


def command_sequence():
    rows = [
        [
            para('<font color="white"><b>1</b></font>', TABLE_HEAD),
            para("<b>ERASE</b>", COMMAND),
            para("이전 비행 데이터 삭제. <b>DONE.</b>이 올 때까지 기다린다.", TABLE_CELL),
            para("약 2-4분", TABLE_CELL_CENTER),
        ],
        [
            para('<font color="white"><b>2</b></font>', TABLE_HEAD),
            para("<b>CALIBRATE</b>", COMMAND),
            para("로켓을 <b>수직·정지</b> 상태로 유지한다. <b>CALIBRATION DONE.</b> 확인.", TABLE_CELL),
            para("약 10-65초", TABLE_CELL_CENTER),
        ],
        [
            para('<font color="white"><b>3</b></font>', TABLE_HEAD),
            para("<b>READY</b>", COMMAND),
            para("최종 발사대 각도에서 정지. <b>ARMED - LAUNCH DETECTION ACTIVE</b>까지 움직이지 않는다.", TABLE_CELL),
            para("보통 40-60초", TABLE_CELL_CENTER),
        ],
        [
            para('<font color="white"><b>4</b></font>', TABLE_HEAD),
            para("<b>DISPLAY</b>", COMMAND),
            para("센서와 단분리 입력 최종 확인. <b>STAGE LOW - JOINED</b>가 필수.", TABLE_CELL),
            para("1회 확인", TABLE_CELL_CENTER),
        ],
        [
            para('<font color="white"><b>5</b></font>', TABLE_HEAD),
            para("<b>명령 없음</b>", COMMAND),
            para("발사 승인 후에는 자동 비행 로직에 맡긴다. 통신 단절이어도 계속 동작한다.", TABLE_CELL),
            para("착륙까지", TABLE_CELL_CENTER),
        ],
    ]
    table = Table(rows, colWidths=[10 * mm, 30 * mm, 110 * mm, 26 * mm])
    table.setStyle(
        TableStyle(
            [
                ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                ("GRID", (0, 0), (-1, -1), 0.55, LINE),
                ("BACKGROUND", (0, 0), (0, -1), BLUE),
                ("ROWBACKGROUNDS", (1, 0), (-1, -1), [WHITE, LIGHT]),
                ("LEFTPADDING", (0, 0), (-1, -1), 7),
                ("RIGHTPADDING", (0, 0), (-1, -1), 7),
                ("TOPPADDING", (0, 0), (-1, -1), 9),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 9),
            ]
        )
    )
    return table


def checkbox(text):
    return para(f"□ {text}", BODY_TIGHT)


def draw_footer(canvas, doc):
    canvas.saveState()
    width, _ = A4
    canvas.setStrokeColor(LINE)
    canvas.setLineWidth(0.5)
    canvas.line(17 * mm, 13 * mm, width - 17 * mm, 13 * mm)
    canvas.setFont("NotoKR", 7)
    canvas.setFillColor(SLATE)
    canvas.drawString(17 * mm, 8.5 * mm, "2026 ALTIS | Final_v5 발사 운용지침 | Rev. 1.0")
    canvas.drawRightString(width - 17 * mm, 8.5 * mm, f"{doc.page} / 6")
    canvas.restoreState()


def build_story():
    story = []

    # Page 1: field quick reference
    story.extend(
        [
            Spacer(1, 7 * mm),
            para("2026 ALTIS", ParagraphStyle(
                "Brand",
                parent=H2,
                fontSize=13,
                leading=16,
                textColor=CYAN,
                spaceAfter=2 * mm,
            )),
            para("Final_v5 발사 운용지침", TITLE),
            para(
                "실제 발사 시 명령 순서, 정상 응답, 자동 비행 상태 및 비상 중단 기준",
                SUBTITLE,
            ),
            data_table(
                [
                    ["문서 개정", "펌웨어 기준", "작성일", "BLE 장치명"],
                    [
                        "Rev. 1.0",
                        "Final_v5 / SHA-256 CC2FD63599F...BF63D0",
                        "2026-07-30",
                        "2026ALTIS",
                    ],
                ],
                [28 * mm, 69 * mm, 37 * mm, 42 * mm],
                header=True,
                font_size=7.7,
            ),
            Spacer(1, 4 * mm),
            callout(
                "안전 원칙",
                "실제 점화 회로가 연결된 뒤에는 <b>TEST PYRO1</b>, <b>TEST PYRO2</b>를 절대 사용하지 않는다. "
                "발사 중 <b>STOP</b>은 점화 출력을 즉시 LOW로 만들고 자동 전개를 중단시킬 수 있으므로 "
                "비상 절차에서만 사용한다.",
                "danger",
            ),
            Spacer(1, 5 * mm),
            para("현장 명령 순서 - 이 순서만 기억", H1),
            command_sequence(),
            Spacer(1, 5 * mm),
            para("발사 직전 GO 기준", H2),
            Table(
                [
                    [checkbox("ARMED 문구 확인"), checkbox("BARO ALT가 0 m 부근")],
                    [checkbox("IMU 가속도 크기가 약 9.8 m/s²"), checkbox("STAGE LOW - JOINED")],
                    [checkbox("기체가 최종 레일 각도에서 정지"), checkbox("인원 철수 및 물리적 안전 절차 완료")],
                ],
                colWidths=[88 * mm, 88 * mm],
                style=TableStyle(
                    [
                        ("BACKGROUND", (0, 0), (-1, -1), GREEN_BG),
                        ("BOX", (0, 0), (-1, -1), 0.8, GREEN),
                        ("INNERGRID", (0, 0), (-1, -1), 0.35, colors.HexColor("#B8D8C6")),
                        ("LEFTPADDING", (0, 0), (-1, -1), 9),
                        ("RIGHTPADDING", (0, 0), (-1, -1), 7),
                        ("TOPPADDING", (0, 0), (-1, -1), 7),
                        ("BOTTOMPADDING", (0, 0), (-1, -1), 7),
                    ]
                ),
            ),
            Spacer(1, 5 * mm),
            callout(
                "발사 승인 이후",
                "<b>추가 명령을 보내지 않는다.</b> BLE 연결이 끊겨도 센서, 점화, 낙하산 전개 및 기록은 보드에서 자동으로 계속된다.",
                "warn",
            ),
            PageBreak(),
        ]
    )

    # Page 2: connection and preparation
    story.extend(
        [
            para("1. 연결 및 발사 전 준비", H1),
            para(
                "명령은 대소문자를 구분하지 않지만 현장 혼선을 막기 위해 아래와 같이 대문자로 전송한다. "
                "한 번에 하나의 명령만 보내고 반드시 응답을 확인한 뒤 다음 단계로 진행한다.",
                BODY,
            ),
            para("통신 설정", H2),
            data_table(
                [
                    ["항목", "설정"],
                    ["BLE 장치명", "<b>2026ALTIS</b>"],
                    ["BLE 방식", "Nordic UART Service (NUS)"],
                    ["RX / 명령", "6E400002-B5A3-F393-E0A9-E50E24DCCA9E / Write"],
                    ["TX / 응답", "6E400003-B5A3-F393-E0A9-E50E24DCCA9E / Notify 구독"],
                    ["USB Serial", "<b>921600 baud</b>, 줄바꿈으로 명령 종료"],
                ],
                [43 * mm, 133 * mm],
            ),
            Spacer(1, 3 * mm),
            callout(
                "부팅 확인",
                "정상 부팅 시 Serial/BLE에 <b>IMU+BARO OK</b>, <b>&gt;&gt;&gt; V5 TWO-TASK READY</b>가 출력된다. "
                "BLE를 늦게 연결해 부팅 문구를 놓쳤다면 <b>DISPLAY</b>로 응답과 센서 갱신을 확인한다.",
                "info",
            ),
            para("1-1. 플래시 초기화", H2),
            data_table(
                [
                    ["운용자 동작", "전송 / 정상 응답", "판정"],
                    [
                        "이전 비행 데이터를 이미 회수했는지 확인한다.",
                        "<b>ERASE</b><br/>ERASING FLASH...<br/><b>DONE.</b>",
                        "DONE. 전까지 다른 명령을 보내지 않는다. 보통 2-4분 소요.",
                    ],
                ],
                [55 * mm, 55 * mm, 66 * mm],
            ),
            Spacer(1, 2 * mm),
            callout(
                "데이터 보존",
                "<b>ERASE는 복구할 수 없다.</b> 이전 비행 데이터가 남아 있다면 먼저 4장의 PARSE 절차로 회수한다.",
                "warn",
            ),
            para("1-2. IMU 캘리브레이션", H2),
            data_table(
                [
                    ["운용자 동작", "전송 / 정상 응답", "실패 시"],
                    [
                        "기체를 수직으로 세우고 진동, 충격, 손 접촉을 없앤다.",
                        "<b>CALIBRATE</b><br/>IMU WARMUP (5s)...<br/>CAL STABLE 1/3 ... 3/3<br/><b>CALIBRATION DONE.</b>",
                        "POSE BAD면 수직 자세 확인. UNSTABLE이면 움직임과 진동 제거. TIMEOUT이면 장착 상태 점검.",
                    ],
                ],
                [55 * mm, 65 * mm, 56 * mm],
            ),
            para("1-3. 최종 발사대 장착", H2),
            Table(
                [
                    [checkbox("캘리브레이션 후 기체를 실제 발사 레일에 장착")],
                    [checkbox("READY를 보내기 전에 최종 기울기와 방향으로 고정")],
                    [checkbox("홀센서가 단 결합 상태 LOW인지 확인")],
                    [checkbox("서보/점화 배선과 물리적 안전장치는 팀 승인 절차에 따라 최종 연결")],
                    [checkbox("이 시점 이후 기체를 들거나 방향을 바꾸지 않음")],
                ],
                colWidths=[176 * mm],
                style=TableStyle(
                    [
                        ("BACKGROUND", (0, 0), (-1, -1), LIGHT),
                        ("BOX", (0, 0), (-1, -1), 0.6, LINE),
                        ("INNERGRID", (0, 0), (-1, -1), 0.3, LINE),
                        ("LEFTPADDING", (0, 0), (-1, -1), 8),
                        ("TOPPADDING", (0, 0), (-1, -1), 6),
                        ("BOTTOMPADDING", (0, 0), (-1, -1), 6),
                    ]
                ),
            ),
            PageBreak(),
        ]
    )

    # Page 3: READY and autonomous flight sequence
    story.extend(
        [
            para("2. READY, 최종 확인 및 자동 비행", H1),
            para("2-1. READY 실행", H2),
            data_table(
                [
                    ["순서", "보드 응답", "운용자 행동"],
                    ["1", "<b>READY</b>", "최종 레일 자세에서 명령 전송"],
                    ["2", "READY ALIGNING...", "움직이지 않음. 약 400개 IMU 샘플로 초기 자세 계산"],
                    ["3", "READY BARO ZEROING...", "기압 기준 고도 0점 설정"],
                    ["4", "READY CONVERGING - WAIT FOR ARMED", "계속 정지. 자이로 바이어스와 기압 기준 수렴 대기"],
                    ["5", "<b>ARMED - LAUNCH DETECTION ACTIVE</b>", "발사 감지 활성화. 다음은 DISPLAY 1회만 수행"],
                ],
                [13 * mm, 72 * mm, 91 * mm],
            ),
            Spacer(1, 3 * mm),
            callout(
                "READY 거부 조건",
                "<b>CALIBRATE REQUIRED</b>이면 CALIBRATE, <b>ERASE REQUIRED</b>이면 ERASE를 먼저 수행한다. "
                "<b>STAGE SEP SENSOR NOT LOW</b>이면 홀센서, 자석, 배선을 고치기 전에는 발사하지 않는다.",
                "warn",
            ),
            para("2-2. DISPLAY 최종 판정", H2),
            data_table(
                [
                    ["출력", "정상 기준"],
                    ["IMU RAW ...", "값이 고정되어 있지 않고 센서가 갱신됨"],
                    ["IMU SI A[...] m/s2", "세 축 벡터 크기가 약 9.8 m/s²"],
                    ["IMU SI ... G[...] dps", "정지 상태에서 각 축이 0 dps 부근"],
                    ["BARO ALT ... m", "0 m 부근. 큰 지속 드리프트가 없어야 함"],
                    ["<b>STAGE LOW - JOINED</b>", "필수 GO 조건. HIGH는 이미 분리로 인식되므로 NO-GO"],
                ],
                [68 * mm, 108 * mm],
            ),
            Spacer(1, 3 * mm),
            callout(
                "최종 명령",
                "DISPLAY 판정이 정상이라면 더 이상 명령을 보내지 않는다. <b>READY를 다시 보내거나 REBOOT하지 않는다.</b>",
                "good",
            ),
            para("2-3. 발사 이후 자동 상태 전이 - 운용자 명령 없음", H2),
            data_table(
                [
                    ["단계", "자동 조건", "주요 응답 / 동작"],
                    [
                        "발사 후보",
                        "축방향 가속도 3 g 이상 1회",
                        "LAUNCH CANDIDATE - VERIFYING 1s/10m<br/>발사 시각 자세, 위치, 속도 초기화",
                    ],
                    [
                        "발사 확정",
                        "1초 후 고도 10 m 초과",
                        "LAUNCH CONFIRMED<br/>발사 전 약 1초 데이터와 이후 로그를 플래시에 저장",
                    ],
                    [
                        "관성 비행",
                        "추력 종료 감지 또는 발사 감지 후 2초",
                        "BO 또는 BO TIMEOUT<br/>COASTING 진입",
                    ],
                    [
                        "2단 점화",
                        "단분리 HIGH 3회 연속 + 자세 45도 이내",
                        "STAGE2 IGN<br/>PYRO1 1초 출력",
                    ],
                    [
                        "어포지 / 드로그",
                        "<b>KF 하강 3회 우선</b><br/>백업: 미점화 COAST 2초 / 점화성공 8.5초",
                        "APG / APG NO IGN TIMEOUT / APG TIMEOUT<br/>이후 DROGUE",
                    ],
                    [
                        "메인 전개",
                        "DESCENT에서 고도 100 m 미만",
                        "MAIN<br/>PYRO2 1초 출력",
                    ],
                    [
                        "착륙",
                        "고도 10 m 미만, 속도 절댓값 1 m/s 미만 10회",
                        "LAND -> 로그 완전 저장 -> STOPPED.",
                    ],
                ],
                [27 * mm, 75 * mm, 74 * mm],
                font_size=7.5,
            ),
            PageBreak(),
        ]
    )

    # Page 4: faults and postflight recovery
    story.extend(
        [
            para("3. 이상 상황 및 착륙 후 회수", H1),
            para("3-1. 현장 이상 대응", H2),
            data_table(
                [
                    ["상황 / 응답", "운용자 조치"],
                    ["BLE 연결 끊김", "보드는 자율 비행을 계속한다. 전원을 끄거나 REBOOT하지 말고 안전 거리에서 재연결만 시도."],
                    ["FALSE LAUNCH REJECTED - RECONVERGING", "실제 발사가 아니면 기체를 건드리지 말고 ARMED 문구가 다시 올 때까지 대기."],
                    ["CALIBRATION TIMEOUT", "STOP 후 장착, 수직 자세, 진동원을 점검하고 CALIBRATE 재수행."],
                    ["KF ALIGN RETRY / BARO UNSTABLE", "기체와 발사대를 정지시키고 대기. 반복되면 STOP 후 센서 장착과 날씨/진동 점검."],
                    ["CMD QUEUE FULL", "명령 반복 전송을 멈추고 응답을 기다린다. 필요하면 안전 상태에서 재연결."],
                    ["ARMED 이후 센서값 비정상", "발사 NO-GO. STOP을 보내 STOPPED. 확인 후 READY부터 다시 수행."],
                ],
                [67 * mm, 109 * mm],
                font_size=7.7,
            ),
            Spacer(1, 3 * mm),
            callout(
                "STOP과 REBOOT",
                "<b>STOP</b>: PYRO1/PYRO2를 LOW로 만들고 로그를 마감한다. ARMED 취소 또는 안전한 회수 후 사용한다.<br/>"
                "<b>REBOOT</b>: 출력 LOW 및 로그 마감 후 보드를 재시작한다. 비행 중 사용 금지. 재부팅 뒤에는 이 부팅에서 CALIBRATE/ERASE가 "
                "완료되지 않은 상태로 돌아간다.",
                "danger",
            ),
            para("3-2. 정상 착륙 후", H2),
            Table(
                [
                    [para("<b>1</b>", TABLE_CELL_CENTER), para("<b>LAND</b> 확인", TABLE_CELL), para("착륙 판정 로그가 기록됨", TABLE_CELL)],
                    [para("<b>2</b>", TABLE_CELL_CENTER), para("<b>STOPPED.</b> 확인", TABLE_CELL), para("RAM 큐와 남은 페이지가 플래시에 완전히 저장됨", TABLE_CELL)],
                    [para("<b>3</b>", TABLE_CELL_CENTER), para("그 뒤 전원 OFF", TABLE_CELL), para("STOPPED. 전에 전원을 끄지 않음", TABLE_CELL)],
                    [para("<b>4</b>", TABLE_CELL_CENTER), para("다음 비행 전 데이터 회수", TABLE_CELL), para("회수 전 ERASE 금지", TABLE_CELL)],
                ],
                colWidths=[12 * mm, 62 * mm, 102 * mm],
                style=TableStyle(
                    [
                        ("GRID", (0, 0), (-1, -1), 0.45, LINE),
                        ("ROWBACKGROUNDS", (0, 0), (-1, -1), [WHITE, LIGHT]),
                        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                        ("LEFTPADDING", (0, 0), (-1, -1), 7),
                        ("RIGHTPADDING", (0, 0), (-1, -1), 7),
                        ("TOPPADDING", (0, 0), (-1, -1), 6),
                        ("BOTTOMPADDING", (0, 0), (-1, -1), 6),
                    ]
                ),
            ),
            para("3-3. 비행 데이터 다운로드", H2),
            data_table(
                [
                    ["순서", "조치"],
                    ["1", "PC와 USB Serial 연결 후 <b>Final_v5/parse/main.py</b> 실행"],
                    ["2", "COM 포트 선택, <b>921600</b> baud, Connect"],
                    ["3", "PARSE (Dump) 버튼 또는 <b>PARSE</b> 명령. DUMP START 확인"],
                    ["4", "32 MB 전체 전송이 끝나 <b>DUMP DONE</b>이 나올 때까지 연결 유지. 약 6-10분 예상"],
                    ["5", "Save BIN으로 원본 저장 후 Save XLSX. 저장 검증 뒤에만 ERASE 수행"],
                ],
                [17 * mm, 159 * mm],
                font_size=7.8,
            ),
            PageBreak(),
            para("3-4. 전체 명령 참조", H1),
            para(
                "아래 표는 Final_v5가 인식하는 전체 명령이다. 실제 발사 순서는 ERASE, CALIBRATE, READY, DISPLAY이며 "
                "나머지는 중단, 복구, 데이터 회수 또는 승인된 지상 기능시험에만 사용한다.",
                BODY,
            ),
            data_table(
                [
                    ["명령", "용도", "사용 시점 / 주의"],
                    ["ERASE", "외부 플래시 전체 삭제", "비행 전 1회. 이전 데이터 회수 후"],
                    ["CALIBRATE", "IMU 바이어스 보정", "수직·정지 상태"],
                    ["READY", "레일 자세 정렬, 기압 0점, ARMED 진입", "최종 발사대 장착 후"],
                    ["DISPLAY", "IMU, BARO, 단분리 입력 표시", "발사 전만 사용"],
                    ["STOP", "출력 LOW, 로그 마감, 운용 중단", "ARMED 취소/안전 회수. 비행 중은 비상 전용"],
                    ["REBOOT", "출력 LOW, 로그 마감, 재시작", "안전 상태의 장애 복구 전용"],
                    ["PARSE", "32 MB 로그를 Serial로 전송", "IDLE 또는 LANDED, USB Serial 필요"],
                    ["TEST SERVO", "드로그 서보 왕복 시험", "지상 기능시험 전용"],
                    ["TEST PYRO1 / 2", "각 점화 출력 1초 시험", "실점화 장치 분리 및 승인된 지상시험에서만"],
                ],
                [36 * mm, 66 * mm, 74 * mm],
                font_size=7.2,
            ),
            Spacer(1, 3 * mm),
            callout(
                "최종 책임",
                "본 문서는 Final_v5 펌웨어 동작 순서의 운용 지침이다. 발사장 안전 규정, 점화 회로 물리적 안전 절차, "
                "현장 책임자의 GO/NO-GO 판단을 대체하지 않는다.",
                "info",
            ),
            Spacer(1, 2 * mm),
            para(
                "코드 기준: src/main.cpp, src/Config.h, src/BLE.cpp, src/MX25Logger.cpp | "
                "펌웨어 SHA-256: CC2FD63599F666DF1E2E2E802C0468FCA622204CD269B00B97D5742E7BBF63D0",
                FOOT,
            ),
            PageBreak(),
        ]
    )

    # Page 6: printable checklist
    story.extend(
        [
            para("4. 인쇄용 발사 체크리스트", H1),
            para("임무명: ____________________   날짜: __________   발사대: __________   운용자: ____________________", BODY),
            Spacer(1, 2 * mm),
            para("A. 전원 및 통신", H2),
            data_table(
                [
                    ["확인", "항목", "기록"],
                    ["□", "보드 전원 ON, 비정상 발열/냄새 없음", ""],
                    ["□", "BLE 2026ALTIS 또는 USB Serial 연결", ""],
                    ["□", "IMU/BARO 응답 확인", ""],
                ],
                [15 * mm, 125 * mm, 36 * mm],
            ),
            para("B. 초기화 및 센서", H2),
            data_table(
                [
                    ["확인", "명령 / 항목", "정상 응답 또는 기록값"],
                    ["□", "ERASE", "DONE."],
                    ["□", "CALIBRATE - 수직·정지", "CALIBRATION DONE."],
                    ["□", "최종 레일 각도 장착 및 고정", "각도: ______ °"],
                    ["□", "홀센서 / 자석 / 단 결합", "LOW / JOINED"],
                ],
                [15 * mm, 92 * mm, 69 * mm],
            ),
            para("C. READY 및 GO/NO-GO", H2),
            data_table(
                [
                    ["확인", "명령 / 항목", "정상 응답 또는 기록값"],
                    ["□", "READY", "ARMED - LAUNCH DETECTION ACTIVE"],
                    ["□", "DISPLAY - BARO", "________ m"],
                    ["□", "DISPLAY - 가속도 크기", "________ m/s²"],
                    ["□", "DISPLAY - 자이로 정지값", "정상 / 재확인"],
                    ["□", "DISPLAY - STAGE", "LOW - JOINED"],
                    ["□", "인원 철수 및 현장 안전 승인", "책임자: __________________"],
                    ["□", "최종 GO", "시각: ____ : ____ : ____"],
                ],
                [15 * mm, 92 * mm, 69 * mm],
            ),
            Spacer(1, 3 * mm),
            callout(
                "GO 이후",
                "명령을 보내지 않는다. BLE가 끊겨도 전원을 유지한다. STOP과 REBOOT은 승인된 비상 절차에서만 사용한다.",
                "danger",
            ),
            para("D. 착륙 및 데이터 회수", H2),
            data_table(
                [
                    ["확인", "항목", "기록"],
                    ["□", "LAND 수신", "시각: ____ : ____ : ____"],
                    ["□", "STOPPED. 수신 후 전원 OFF", "시각: ____ : ____ : ____"],
                    ["□", "PARSE / DUMP DONE", "BIN 파일명: __________________"],
                    ["□", "BIN/XLSX 확인 후 다음 비행 ERASE 승인", "확인자: __________________"],
                ],
                [15 * mm, 92 * mm, 69 * mm],
            ),
            Spacer(1, 2 * mm),
            Table(
                [
                    [para("<b>비고 / 이상 사항</b>", TABLE_CELL)],
                    [para("<br/><br/><br/>", BODY)],
                ],
                colWidths=[176 * mm],
                style=TableStyle(
                    [
                        ("BACKGROUND", (0, 0), (-1, 0), LIGHT),
                        ("BOX", (0, 0), (-1, -1), 0.6, LINE),
                        ("LINEBELOW", (0, 0), (-1, 0), 0.6, LINE),
                        ("LEFTPADDING", (0, 0), (-1, -1), 8),
                        ("RIGHTPADDING", (0, 0), (-1, -1), 8),
                        ("TOPPADDING", (0, 0), (-1, -1), 6),
                        ("BOTTOMPADDING", (0, 0), (-1, -1), 6),
                    ]
                ),
            ),
        ]
    )
    return story


def main():
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    doc = SimpleDocTemplate(
        str(OUTPUT),
        pagesize=A4,
        leftMargin=17 * mm,
        rightMargin=17 * mm,
        topMargin=16 * mm,
        bottomMargin=18 * mm,
        title="2026 ALTIS Final_v5 발사 운용지침",
        author="2026 ALTIS",
        subject="Final_v5 실제 발사 명령 및 운용 절차",
    )
    doc.build(build_story(), onFirstPage=draw_footer, onLaterPages=draw_footer)
    print(OUTPUT)


if __name__ == "__main__":
    main()
