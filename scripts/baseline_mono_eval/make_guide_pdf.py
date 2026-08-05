#!/usr/bin/env python3
"""Generate a polished PDF version of the baseline evaluation guide."""

import os
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_JUSTIFY, TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import cm, mm
from reportlab.platypus import (
    SimpleDocTemplate,
    Paragraph,
    Spacer,
    Table,
    TableStyle,
    Preformatted,
    KeepTogether,
    ListFlowable,
    ListItem,
    HRFlowable,
    PageBreak,
)
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont

OUT = os.environ.get(
    "GUIDE_PDF",
    "/results_host/BASELINE_EVAL_GUIDE.pdf"
    if os.path.isdir("/results_host")
    else "/home/aze-pc-0266/workspace/eval/baseline_mono/BASELINE_EVAL_GUIDE.pdf",
)

BRAND = colors.HexColor("#1B4F72")
ACCENT = colors.HexColor("#2874A6")
SOFT = colors.HexColor("#F4F6F7")
CODEBG = colors.HexColor("#EEF2F5")
OK = colors.HexColor("#1E8449")
WARN = colors.HexColor("#B9770E")
LINE = colors.HexColor("#D5D8DC")


def styles():
    base = getSampleStyleSheet()
    s = {
        "title": ParagraphStyle(
            "title",
            parent=base["Title"],
            fontName="Helvetica-Bold",
            fontSize=22,
            textColor=BRAND,
            alignment=TA_CENTER,
            spaceAfter=4,
        ),
        "subtitle": ParagraphStyle(
            "subtitle",
            parent=base["Normal"],
            fontName="Helvetica",
            fontSize=13,
            textColor=ACCENT,
            alignment=TA_CENTER,
            spaceAfter=6,
        ),
        "meta": ParagraphStyle(
            "meta",
            parent=base["Normal"],
            fontSize=10,
            textColor=colors.HexColor("#566573"),
            alignment=TA_CENTER,
            spaceAfter=14,
        ),
        "h1": ParagraphStyle(
            "h1",
            parent=base["Heading1"],
            fontName="Helvetica-Bold",
            fontSize=14,
            textColor=BRAND,
            spaceBefore=14,
            spaceAfter=8,
        ),
        "h2": ParagraphStyle(
            "h2",
            parent=base["Heading2"],
            fontName="Helvetica-Bold",
            fontSize=12,
            textColor=ACCENT,
            spaceBefore=10,
            spaceAfter=6,
        ),
        "body": ParagraphStyle(
            "body",
            parent=base["Normal"],
            fontName="Helvetica",
            fontSize=10,
            leading=14,
            alignment=TA_JUSTIFY,
            spaceAfter=6,
        ),
        "bullet": ParagraphStyle(
            "bullet",
            parent=base["Normal"],
            fontName="Helvetica",
            fontSize=10,
            leading=13,
            leftIndent=8,
        ),
        "code": ParagraphStyle(
            "code",
            parent=base["Code"],
            fontName="Courier",
            fontSize=8.2,
            leading=11,
            backColor=CODEBG,
            borderPadding=6,
        ),
        "path": ParagraphStyle(
            "path",
            parent=base["Normal"],
            fontName="Courier",
            fontSize=8.5,
            textColor=BRAND,
            leading=11,
        ),
        "note": ParagraphStyle(
            "note",
            parent=base["Normal"],
            fontName="Helvetica",
            fontSize=9.5,
            leading=13,
            textColor=colors.HexColor("#1C2833"),
        ),
        "footer": ParagraphStyle(
            "footer",
            parent=base["Normal"],
            fontSize=8,
            textColor=colors.gray,
            alignment=TA_CENTER,
        ),
    }
    return s


def callout(text, border=BRAND):
    data = [[Paragraph(text, styles()["note"])]]
    t = Table(data, colWidths=[17.2 * cm])
    t.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, -1), SOFT),
                ("BOX", (0, 0), (-1, -1), 1.2, border),
                ("LEFTPADDING", (0, 0), (-1, -1), 10),
                ("RIGHTPADDING", (0, 0), (-1, -1), 10),
                ("TOPPADDING", (0, 0), (-1, -1), 8),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 8),
            ]
        )
    )
    return t


def code_block(text):
    data = [[Preformatted(text.rstrip("\n"), styles()["code"])]]
    t = Table(data, colWidths=[17.2 * cm])
    t.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, -1), CODEBG),
                ("BOX", (0, 0), (-1, -1), 0.5, LINE),
                ("LEFTPADDING", (0, 0), (-1, -1), 8),
                ("RIGHTPADDING", (0, 0), (-1, -1), 8),
                ("TOPPADDING", (0, 0), (-1, -1), 6),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 6),
            ]
        )
    )
    return t


def nice_table(header, rows, widths):
    s = styles()
    head = [Paragraph(f"<b>{h}</b>", s["note"]) for h in header]
    body = []
    for r in rows:
        body.append([Paragraph(str(c), s["note"]) for c in r])
    data = [head] + body
    t = Table(data, colWidths=widths, repeatRows=1)
    style_cmds = [
        ("BACKGROUND", (0, 0), (-1, 0), BRAND),
        ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
        ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"),
        ("ALIGN", (0, 0), (-1, 0), "CENTER"),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
        ("GRID", (0, 0), (-1, -1), 0.4, LINE),
        ("LEFTPADDING", (0, 0), (-1, -1), 6),
        ("RIGHTPADDING", (0, 0), (-1, -1), 6),
        ("TOPPADDING", (0, 0), (-1, -1), 5),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 5),
    ]
    for i in range(1, len(data)):
        if i % 2 == 0:
            style_cmds.append(("BACKGROUND", (0, i), (-1, i), SOFT))
    t.setStyle(TableStyle(style_cmds))
    return t


def header_footer(canvas, doc):
    canvas.saveState()
    canvas.setStrokeColor(BRAND)
    canvas.setLineWidth(0.8)
    canvas.line(1.8 * cm, A4[1] - 1.3 * cm, A4[0] - 1.8 * cm, A4[1] - 1.3 * cm)
    canvas.setFont("Helvetica-Bold", 8)
    canvas.setFillColor(BRAND)
    canvas.drawString(1.8 * cm, A4[1] - 1.1 * cm, "Baseline Mono MSCKF Evaluation")
    canvas.setFont("Helvetica", 8)
    canvas.setFillColor(colors.gray)
    canvas.drawRightString(A4[0] - 1.8 * cm, A4[1] - 1.1 * cm, "OpenVINS / PO-MSCKF")
    canvas.line(1.8 * cm, 1.4 * cm, A4[0] - 1.8 * cm, 1.4 * cm)
    canvas.drawCentredString(A4[0] / 2.0, 0.9 * cm, f"Page {doc.page}")
    canvas.restoreState()


def build():
    s = styles()
    story = []

    story.append(Paragraph("Baseline Mono MSCKF", s["title"]))
    story.append(Paragraph("Evaluation &amp; Reproducibility Guide", s["subtitle"]))
    story.append(Paragraph("OpenVINS — Step 1.5 before PO-MSCKF", s["meta"]))
    story.append(Spacer(1, 4))
    story.append(
        callout(
            "<b>Purpose.</b> Lock a fair mono-camera MSCKF baseline on EuRoC Machine Hall "
            "and UZH-FPV outdoor sequences. Later PO-MSCKF runs must reuse this exact protocol."
        )
    )

    story.append(Paragraph("1. Protocol Summary", s["h1"]))
    story.append(
        nice_table(
            ["Item", "Value"],
            [
                ["Estimator", "OpenVINS MSCKF (serial.launch)"],
                ["Camera mode", "Mono: max_cameras:=1, use_stereo:=false (left / cam0)"],
                ["Alignment", "posyaw via ov_eval error_singlerun"],
                ["Table metrics", "Trans. RMSE (m), Ori. RMSE (deg), ms/frame, FPS, CPU %"],
                ["Error plots", "error_dataset-style RMSE (ori + pos vs time)"],
                ["Trajectories", "XY and Z vs ground truth"],
            ],
            [4.2 * cm, 13.0 * cm],
        )
    )
    story.append(Spacer(1, 8))
    story.append(
        callout(
            "<b>Important rules</b><br/>"
            "• Do <b>not</b> run a separate <font face='Courier'>rosbag play</font> with "
            "<font face='Courier'>serial.launch</font> — the serial node already reads the bag.<br/>"
            "• Do <b>not</b> average log <font face='Courier'>(xx hz)</font> lines for FPS — that is print rate.<br/>"
            "• Correct FPS = 1 / mean_total from <font face='Courier'>timing.txt</font> "
            "(requires <font face='Courier'>dotime:=true</font>).<br/>"
            "• CPU = mean/max % of process <font face='Courier'>ros1_serial_msckf</font> "
            "(100% ≈ one full core).",
            border=WARN,
        )
    )

    story.append(Paragraph("2. Workspace Layout", s["h1"]))
    story.append(Paragraph("<b>Tools</b>", s["h2"]))
    story.append(Paragraph("/home/aze-pc-0266/workspace/eval/baseline_mono/", s["path"]))
    story.append(Paragraph("Mirrored for git: open_vins/scripts/baseline_mono_eval/", s["path"]))
    story.append(Paragraph("<b>Results</b>", s["h2"]))
    story.append(Paragraph("/home/aze-pc-0266/results/baseline_mono_msckf/", s["path"]))
    story.append(Spacer(1, 6))
    story.append(
        nice_table(
            ["File", "Role"],
            [
                ["run_baseline_mono.sh", "Run all mono sequences + CPU sampling"],
                ["recompute_baseline_metrics.py", "Official ATE → CSV / TABLE"],
                ["make_hires_pdf.py", "Large PDF report (table + plots)"],
                ["BASELINE_EVAL_GUIDE.md / .tex / .pdf", "This guide (source + PDF)"],
            ],
            [6.5 * cm, 10.7 * cm],
        )
    )
    story.append(Paragraph("<b>Docker mounts</b>", s["h2"]))
    story.append(
        nice_table(
            ["Host", "Container", "Env"],
            [
                ["$DOCKER_CATKINWS", "/catkin_ws", "—"],
                ["$DOCKER_DATASETS", "/datasets", "DATASETS_DIR"],
                ["~/results", "/results", "RESULTS_DIR"],
                ["~/workspace/eval", "/workspace_eval", "—"],
            ],
            [5.5 * cm, 5.5 * cm, 6.2 * cm],
        )
    )
    story.append(Spacer(1, 6))
    story.append(
        code_block(
            "export DOCKER_CATKINWS=/home/aze-pc-0266/workspace/catkin_ws_ov\n"
            "export DOCKER_DATASETS=/home/aze-pc-0266/datasets"
        )
    )

    story.append(Paragraph("3. Sequences", s["h1"]))
    story.append(Paragraph("EuRoC Machine Hall", s["h2"]))
    story.append(
        nice_table(
            ["Name", "Config", "bag_start"],
            [
                ["MH_01_easy", "euroc_mav", "40"],
                ["MH_02_easy", "euroc_mav", "35"],
                ["MH_03_medium", "euroc_mav", "17.5"],
                ["MH_04_difficult", "euroc_mav", "15"],
                ["MH_05_difficult", "euroc_mav", "15"],
            ],
            [5.5 * cm, 6.0 * cm, 5.7 * cm],
        )
    )
    story.append(Spacer(1, 4))
    story.append(code_block("/datasets/machine_hall/<SEQ>/<SEQ>.bag\n/datasets/machine_hall/<SEQ>/<SEQ>/mav0/state_groundtruth_estimate0/data.txt"))

    story.append(Paragraph("UZH-FPV Outdoor", s["h2"]))
    story.append(
        nice_table(
            ["Name", "Config", "bag_start"],
            [
                ["outdoor_forward_1", "uzhfpv_outdoor", "0"],
                ["outdoor_forward_3", "uzhfpv_outdoor", "0"],
                ["outdoor_forward_5", "uzhfpv_outdoor", "0"],
                ["outdoor_45_1", "uzhfpv_outdoor_45", "0"],
            ],
            [5.5 * cm, 6.0 * cm, 5.7 * cm],
        )
    )
    story.append(Spacer(1, 6))
    story.append(
        callout(
            "For the 45° sequence always use config <font face='Courier'><b>uzhfpv_outdoor_45</b></font> "
            "(not the forward config).",
            border=ACCENT,
        )
    )

    story.append(Paragraph("4. Full Baseline Pipeline (Recommended)", s["h1"]))
    story.append(Paragraph("Step A — Run all estimator sequences", s["h2"]))
    story.append(
        code_block(
            "mkdir -p /home/aze-pc-0266/results/baseline_mono_msckf\n\n"
            "docker run --rm --net=host \\\n"
            "  -e MPLBACKEND=Agg \\\n"
            "  -e RESULTS_DIR=/results/baseline_mono_msckf \\\n"
            "  -e DATASETS_DIR=/datasets \\\n"
            "  --mount type=bind,source=\"$DOCKER_CATKINWS\",target=/catkin_ws \\\n"
            "  --mount type=bind,source=\"$DOCKER_DATASETS\",target=/datasets \\\n"
            "  --mount type=bind,source=/home/aze-pc-0266/results,target=/results \\\n"
            "  --mount type=bind,source=/home/aze-pc-0266/workspace/eval,target=/workspace_eval \\\n"
            "  ov_ros1_20_04:latest \\\n"
            "  bash /workspace_eval/baseline_mono/run_baseline_mono.sh"
        )
    )
    story.append(Paragraph("Outputs per sequence: estimate.txt, timing.txt, cpu.txt, run.log.", s["body"]))

    story.append(Paragraph("Step B — Recompute official ATE metrics", s["h2"]))
    story.append(
        code_block(
            "docker run --rm --net=host \\\n"
            "  -e MPLBACKEND=Agg \\\n"
            "  -e RESULTS_DIR=/results/baseline_mono_msckf \\\n"
            "  -e DATASETS_DIR=/datasets \\\n"
            "  --mount type=bind,source=\"$DOCKER_CATKINWS\",target=/catkin_ws \\\n"
            "  --mount type=bind,source=\"$DOCKER_DATASETS\",target=/datasets \\\n"
            "  --mount type=bind,source=/home/aze-pc-0266/results,target=/results \\\n"
            "  --mount type=bind,source=/home/aze-pc-0266/workspace/eval,target=/workspace_eval \\\n"
            "  ov_ros1_20_04:latest \\\n"
            "  bash -lc 'source /opt/ros/noetic/setup.bash; source /catkin_ws/devel/setup.bash; \\\n"
            "    python3 /workspace_eval/baseline_mono/recompute_baseline_metrics.py'"
        )
    )

    story.append(Paragraph("Step C — Build the large PDF report", s["h2"]))
    story.append(
        code_block(
            "docker run --rm \\\n"
            "  -e MPLBACKEND=Agg \\\n"
            "  -e RESULTS_DIR=/results/baseline_mono_msckf \\\n"
            "  -e DATASETS_DIR=/datasets \\\n"
            "  --mount type=bind,source=\"$DOCKER_DATASETS\",target=/datasets \\\n"
            "  --mount type=bind,source=/home/aze-pc-0266/results,target=/results \\\n"
            "  --mount type=bind,source=/home/aze-pc-0266/workspace/eval,target=/workspace_eval \\\n"
            "  ov_ros1_20_04:latest \\\n"
            "  bash -lc 'python3 /workspace_eval/baseline_mono/make_hires_pdf.py'"
        )
    )
    story.append(Spacer(1, 6))
    story.append(
        callout(
            "<b>Report PDF</b><br/>"
            "<font face='Courier'>~/results/baseline_mono_msckf/baseline_mono_msckf_report.pdf</font>",
            border=OK,
        )
    )

    story.append(Paragraph("5. Manual Single-Sequence Example", s["h1"]))
    story.append(Paragraph("Terminal 1 — ROS master", s["h2"]))
    story.append(
        code_block(
            "ov_docker ov_ros1_20_04 bash\n"
            "source /opt/ros/noetic/setup.bash\n"
            "source /catkin_ws/devel/setup.bash\n"
            "roscore"
        )
    )
    story.append(Paragraph("Terminal 2 — EuRoC MH_01 (mono)", s["h2"]))
    story.append(
        code_block(
            "ov_docker ov_ros1_20_04 bash\n"
            "source /opt/ros/noetic/setup.bash\n"
            "source /catkin_ws/devel/setup.bash\n\n"
            "SEQ=MH_01_easy\n"
            "OUT=/results/baseline_mono_msckf/${SEQ}\n"
            "mkdir -p \"$OUT\"\n\n"
            "roslaunch ov_msckf serial.launch \\\n"
            "  config:=euroc_mav \\\n"
            "  dataset:=${SEQ} \\\n"
            "  bag:=/datasets/machine_hall/${SEQ}/${SEQ}.bag \\\n"
            "  bag_start:=40 \\\n"
            "  max_cameras:=1 \\\n"
            "  use_stereo:=false \\\n"
            "  dosave:=true \\\n"
            "  dotime:=true \\\n"
            "  path_est:=${OUT}/estimate.txt \\\n"
            "  path_time:=${OUT}/timing.txt"
        )
    )
    story.append(Paragraph("Accuracy + timing after the run", s["h2"]))
    story.append(
        code_block(
            "GT=/datasets/machine_hall/${SEQ}/${SEQ}/mav0/state_groundtruth_estimate0/data.txt\n"
            "rosrun ov_eval error_singlerun posyaw \"$GT\" ${OUT}/estimate.txt\n"
            "rosrun ov_eval plot_trajectories posyaw \"$GT\" ${OUT}/estimate.txt\n"
            "rosrun ov_eval timing_flamegraph ${OUT}/timing.txt\n"
            "# ms/frame = mean_time(total) * 1000\n"
            "# FPS     = 1 / mean_time(total)"
        )
    )

    story.append(Paragraph("6. Metrics Definitions", s["h1"]))
    story.append(
        nice_table(
            ["Metric", "Meaning / source"],
            [
                ["Trans. RMSE", "ATE translation RMSE after posyaw (error_singlerun)"],
                ["Ori. RMSE", "ATE orientation RMSE in degrees (same tool)"],
                ["ms/frame", "Mean of timing.txt total column × 1000"],
                ["FPS", "1 / mean_total — offline serial throughput"],
                ["CPU mean/max", "psutil sample of ros1_serial_msckf during the run"],
            ],
            [4.0 * cm, 13.2 * cm],
        )
    )
    story.append(Spacer(1, 6))
    story.append(Paragraph("RPE columns are not required for this deliverable and are omitted from the report table.", s["body"]))

    story.append(Paragraph("7. Output Layout", s["h1"]))
    story.append(
        code_block(
            "~/results/baseline_mono_msckf/\n"
            "  TABLE.md\n"
            "  summary.csv\n"
            "  baseline_mono_msckf_report.pdf\n"
            "  MH_01_easy/\n"
            "    estimate.txt  timing.txt  cpu.txt  metrics.txt\n"
            "    rmse.png  run.log  ov_eval_error_singlerun.txt\n"
            "  ...\n"
            "  outdoor_45_1/"
        )
    )

    story.append(Paragraph("8. Next: PO-MSCKF Comparison", s["h1"]))
    story.append(
        callout(
            "Repeat this <b>same</b> protocol with <font face='Courier'>use_pose_only_update:=true</font> "
            "(after the PO updater is implemented), write results to a separate folder "
            "(e.g. <font face='Courier'>baseline_mono_po_msckf/</font>), then compare tables/PDFs.<br/><br/>"
            "Do <b>not</b> change mono/stereo mode, bag starts, alignment, or timing method "
            "between MSCKF and PO runs.",
            border=BRAND,
        )
    )
    story.append(Spacer(1, 18))
    story.append(Paragraph("OpenVINS PO-MSCKF project — baseline evaluation protocol", s["footer"]))

    # Resolve output path for host or container
    candidates = [
        os.environ.get("GUIDE_PDF"),
        "/workspace_eval/baseline_mono/BASELINE_EVAL_GUIDE.pdf",
        "/home/aze-pc-0266/workspace/eval/baseline_mono/BASELINE_EVAL_GUIDE.pdf",
    ]
    out = next((p for p in candidates if p), candidates[-1])
    os.makedirs(os.path.dirname(out), exist_ok=True)

    doc = SimpleDocTemplate(
        out,
        pagesize=A4,
        leftMargin=1.8 * cm,
        rightMargin=1.8 * cm,
        topMargin=1.8 * cm,
        bottomMargin=1.8 * cm,
        title="Baseline Mono MSCKF Evaluation Guide",
        author="Zackhir",
    )
    doc.build(story, onFirstPage=header_footer, onLaterPages=header_footer)
    print("Wrote", out)


if __name__ == "__main__":
    build()
