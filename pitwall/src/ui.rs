//! ratatui rendering of the dashboard: two board panels on top, a scrolling
//! event log in the middle, and a status/command bar at the bottom.

use std::time::Instant;

use ratatui::layout::{Alignment, Constraint, Layout, Position, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, Clear, Gauge, Paragraph, Sparkline, Wrap};
use ratatui::Frame;

use crate::app::{
    board_kind_str, link_color, link_str, AppState, BoardTelemetry, BUTTONS, HELP,
};

pub fn draw(frame: &mut Frame, app: &AppState) {
    let now = Instant::now();
    let root = Layout::vertical([
        Constraint::Percentage(52), // board panels
        Constraint::Min(4),         // event log
        Constraint::Length(3),      // command/status bar
    ])
    .split(frame.area());

    let panels = Layout::horizontal([Constraint::Percentage(50), Constraint::Percentage(50)])
        .split(root[0]);

    render_board(frame, panels[0], "Controller", &app.controller, false, now);
    render_board(frame, panels[1], "Vehicle", &app.vehicle, true, now);
    render_log(frame, root[1], app);
    render_bar(frame, root[2], app);

    if app.show_help {
        render_help(frame);
    }
}

fn render_help(frame: &mut Frame) {
    let area = centered_rect(frame.area(), 60, HELP.len() as u16 + 2);
    frame.render_widget(Clear, area);
    let lines: Vec<Line> = HELP
        .iter()
        .map(|s| Line::from(Span::raw(*s)))
        .collect();
    let block = Block::default()
        .borders(Borders::ALL)
        .title(Span::styled(
            " Help ",
            Style::default().add_modifier(Modifier::BOLD),
        ))
        .style(Style::default().fg(Color::White));
    frame.render_widget(Paragraph::new(lines).block(block), area);
}

/// A rectangle of the given width/height (in cells) centred within `area`,
/// clamped to fit.
fn centered_rect(area: Rect, width: u16, height: u16) -> Rect {
    let w = width.min(area.width);
    let h = height.min(area.height);
    Rect {
        x: area.x + (area.width - w) / 2,
        y: area.y + (area.height - h) / 2,
        width: w,
        height: h,
    }
}

fn render_board(
    frame: &mut Frame,
    area: Rect,
    name: &str,
    b: &BoardTelemetry,
    is_vehicle: bool,
    now: Instant,
) {
    let stale = b.is_stale(now);
    let border_color = if stale { Color::DarkGray } else { Color::White };
    let block = Block::default()
        .borders(Borders::ALL)
        .border_style(Style::default().fg(border_color))
        .title(Span::styled(
            format!(" {name} "),
            Style::default().add_modifier(Modifier::BOLD),
        ));
    let inner = block.inner(area);
    frame.render_widget(block, area);

    // Rows: link | joyX spark | joyY spark | buttons | (vehicle: gauge + spark)
    let rows = if is_vehicle {
        Layout::vertical([
            Constraint::Length(1), // link
            Constraint::Length(1), // received packet summary
            Constraint::Length(1), // buttons
            Constraint::Length(1), // motor label + gauge
            Constraint::Min(1),    // motor sparkline
            Constraint::Length(1), // current label
            Constraint::Min(1),    // current sparkline
        ])
        .split(inner)
    } else {
        Layout::vertical([
            Constraint::Length(1), // link
            Constraint::Length(1), // buttons
            Constraint::Length(1), // joyX label
            Constraint::Min(1),    // joyX sparkline
            Constraint::Length(1), // joyY label
            Constraint::Min(1),    // joyY sparkline
        ])
        .split(inner)
    };

    let link_span = Span::styled(
        format!(
            "Link: ● {}",
            b.link.map(|s| link_str(&s)).unwrap_or("—")
        ),
        Style::default().fg(link_color(b.link)),
    );
    frame.render_widget(Paragraph::new(Line::from(link_span)), rows[0]);

    if is_vehicle {
        let (x, y) = (fmt_opt(b.joy_x), fmt_opt(b.joy_y));
        frame.render_widget(
            Paragraph::new(format!("Rx pkt:  x={x}  y={y}")),
            rows[1],
        );
        frame.render_widget(buttons_line(b.buttons), rows[2]);

        let duty = b.motor_duty.unwrap_or(0);
        let ratio = ((duty as f64 + 100.0) / 200.0).clamp(0.0, 1.0);
        let duty_color = if duty > 0 {
            Color::Green
        } else if duty < 0 {
            Color::Magenta
        } else {
            Color::DarkGray
        };
        let gauge = Gauge::default()
            .ratio(ratio)
            .label(format!("Motor {duty:+}%"))
            .gauge_style(Style::default().fg(duty_color));
        frame.render_widget(gauge, rows[3]);
        render_spark(
            frame,
            rows[4],
            &b.hist_duty,
            200,
            duty_color,
            stale,
            "no data — enable with: peer remote_tele on",
        );

        // IBT-2 current sense: forward loads R_IS, reverse loads L_IS.
        let r_a = b.cur_r_ma.map(|ma| ma as f64 / 1000.0);
        let l_a = b.cur_l_ma.map(|ma| ma as f64 / 1000.0);
        frame.render_widget(
            Paragraph::new(format!("Curr: R={}  L={}", fmt_amps(r_a), fmt_amps(l_a))),
            rows[5],
        );
        render_spark(
            frame,
            rows[6],
            &b.hist_current,
            30_000, // mA — matches the ~28 A ADC ceiling
            Color::Yellow,
            stale,
            "no data — enable with: peer remote_tele on",
        );
    } else {
        frame.render_widget(buttons_line(b.buttons), rows[1]);
        frame.render_widget(
            Paragraph::new(format!("Joy X: {}", fmt_opt(b.joy_x))),
            rows[2],
        );
        render_spark(frame, rows[3], &b.hist_x, 255, Color::Cyan, stale, "waiting…");
        frame.render_widget(
            Paragraph::new(format!("Joy Y: {}", fmt_opt(b.joy_y))),
            rows[4],
        );
        render_spark(frame, rows[5], &b.hist_y, 255, Color::Cyan, stale, "waiting…");
    }
}

#[allow(clippy::too_many_arguments)]
fn render_spark(
    frame: &mut Frame,
    area: Rect,
    hist: &std::collections::VecDeque<u64>,
    max: u64,
    color: Color,
    stale: bool,
    empty_hint: &str,
) {
    if hist.is_empty() {
        let hint = Paragraph::new(empty_hint)
            .style(Style::default().fg(Color::DarkGray).add_modifier(Modifier::ITALIC));
        frame.render_widget(hint, area);
        return;
    }
    let data: Vec<u64> = hist.iter().copied().collect();
    let color = if stale { Color::DarkGray } else { color };
    let spark = Sparkline::default()
        .data(&data)
        .max(max)
        .style(Style::default().fg(color));
    frame.render_widget(spark, area);
}

fn buttons_line(buttons: u8) -> Paragraph<'static> {
    let mut spans = vec![Span::raw("Btn: ")];
    for (name, mask) in BUTTONS {
        let on = buttons & mask != 0;
        let style = if on {
            Style::default().fg(Color::Green).add_modifier(Modifier::BOLD)
        } else {
            Style::default().fg(Color::DarkGray)
        };
        spans.push(Span::styled(format!("{name} "), style));
    }
    Paragraph::new(Line::from(spans))
}

fn render_log(frame: &mut Frame, area: Rect, app: &AppState) {
    let block = Block::default().borders(Borders::ALL).title(" Log ");
    let inner = block.inner(area);
    frame.render_widget(block, area);

    let capacity = inner.height as usize;
    let start = app.log.len().saturating_sub(capacity);
    let lines: Vec<Line> = app
        .log
        .iter()
        .skip(start)
        .map(|(text, color)| Line::from(Span::styled(text.clone(), Style::default().fg(*color))))
        .collect();
    frame.render_widget(Paragraph::new(lines), inner);
}

fn render_bar(frame: &mut Frame, area: Rect, app: &AppState) {
    let (conn_text, conn_color) = if app.connected {
        ("● connected", Color::Green)
    } else {
        ("● disconnected", Color::Red)
    };
    let gw = app
        .gateway
        .map(|k| board_kind_str(&k))
        .unwrap_or("?");
    let title = Line::from(vec![
        Span::raw(format!(" {} · gw={gw} · ", app.port_name)),
        Span::styled(conn_text, Style::default().fg(conn_color)),
        Span::raw(format!(
            " · err={} · Enter=send  F1=help  Esc=quit ",
            app.error_count
        )),
    ]);
    let block = Block::default()
        .borders(Borders::ALL)
        .title(title)
        .title_alignment(Alignment::Left);
    let inner = block.inner(area);
    frame.render_widget(block, area);

    let prompt = format!("> {}", app.input);
    frame.render_widget(
        Paragraph::new(prompt.clone()).wrap(Wrap { trim: false }),
        inner,
    );
    // Place the cursor at the end of the input.
    let cursor_x = inner.x + 2 + app.input.chars().count() as u16;
    if cursor_x < inner.x + inner.width {
        frame.set_cursor_position(Position::new(cursor_x, inner.y));
    }
}

fn fmt_opt(v: Option<u8>) -> String {
    v.map(|n| n.to_string()).unwrap_or_else(|| "—".into())
}

fn fmt_amps(v: Option<f64>) -> String {
    v.map(|a| format!("{a:.2} A")).unwrap_or_else(|| "—".into())
}

#[cfg(test)]
mod tests {
    use super::*;
    use common_host_proto::{BoardKind, BoardToHost};
    use ratatui::backend::TestBackend;
    use ratatui::Terminal;

    #[test]
    fn draw_renders_without_panicking() {
        let mut app = AppState::new("/dev/ttyACM0".into());
        app.gateway = Some(BoardKind::Vehicle);
        app.connected = true;
        app.ingest(&BoardToHost::Pong { version: 1, board: BoardKind::Vehicle });
        app.ingest(&BoardToHost::MotorState { duty: -30 });
        app.ingest(&BoardToHost::EspNowLinkState(
            common_host_proto::LinkStateKind::Alive,
        ));
        app.input = "motor_pwm 40".into();

        let mut terminal = Terminal::new(TestBackend::new(80, 24)).unwrap();
        terminal.draw(|frame| draw(frame, &app)).unwrap();

        let text = terminal.backend().buffer().content().iter().map(|c| c.symbol()).collect::<String>();
        assert!(text.contains("Controller"));
        assert!(text.contains("Vehicle"));
        assert!(text.contains("motor_pwm 40"));

        // Help popup renders without panicking and shows the tunnel tip.
        app.show_help = true;
        terminal.draw(|frame| draw(frame, &app)).unwrap();
        let help = terminal.backend().buffer().content().iter().map(|c| c.symbol()).collect::<String>();
        assert!(help.contains("Help"));
        assert!(help.contains("remote_tele"));
    }
}
