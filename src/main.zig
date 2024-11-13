const std = @import("std");
const rl = @import("raylib");
const rg = @import("raygui");
const m = std.math;

const Pos = rl.Vector2;
const p = rl.Vector2.init;

const screenWidth = 1200;
const screenHeight = 1000;

const GRID_SIZE: u32 = 501; //127;
const CELL_SIZE: u32 = 4; //32;
const LINE_SIZE: u32 = 1; //4;

const PATH_COLOR = rl.Color.init(150, 150, 150, 0xFF);
const WALL_COLOR = rl.Color.init(0x1A, 0x1A, 0x1A, 0xFF);
const SOLUTION_COLOR = rl.Color.init(100, 100, 150, 0xFF);
const BACKGROUND_COLOR = rl.Color.init(0x1A, 0x1A, 0x2B, 0xFF);
const LINE_COLOR = rl.Color.init(66, 69, 73, 0xFF);
const FINISH_COLOR = rl.Color.init(150, 0, 0, 0xFF);
const START_COLOR = rl.Color.init(0, 150, 0, 0xFF);

const SPEED_FACTOR: u32 = 5;

const CellState = enum {
    path,
    wall,
    solution,
    finish,
    start,

    pub fn isPath(self: CellState) bool {
        return self == .path or self == .solution or self == .start;
    }
};

const MazeState = enum {
    Static,
    Solving,
    Generating,
};

const MazeAlgorithm = enum {
    DFS,
    DFST,
};

const Direction = enum {
    UP,
    RIGHT,
    DOWN,
    LEFT,

    pub fn getRight(self: Direction) Direction {
        return switch (self) {
            .UP => .RIGHT,
            .RIGHT => .DOWN,
            .DOWN => .LEFT,
            .LEFT => .UP,
        };
    }

    pub fn getLeft(self: Direction) Direction {
        return switch (self) {
            .UP => .LEFT,
            .LEFT => .DOWN,
            .DOWN => .RIGHT,
            .RIGHT => .UP,
        };
    }

    pub fn xy(self: Direction) [2]i32 {
        return switch (self) {
            .UP => [2]i32{ 0, 1 },
            .LEFT => [2]i32{ -1, 0 },
            .DOWN => [2]i32{ 0, -1 },
            .RIGHT => [2]i32{ 1, 0 },
        };
    }
};

const Maze = struct {
    grid: [GRID_SIZE][GRID_SIZE]CellState,
    state: MazeState,
    alg: MazeAlgorithm,
    rnd: std.Random.DefaultPrng,
    direction: Direction,
    speed: u32,
    x: u32,
    y: u32,

    pub fn init() Maze {
        return Maze{
            .grid = [_][GRID_SIZE]CellState{[_]CellState{.wall} ** GRID_SIZE} ** GRID_SIZE,
            .state = .Static,
            .alg = .DFS,
            .rnd = std.Random.DefaultPrng.init(@as(u64, @bitCast(std.time.milliTimestamp()))),
            .direction = .UP,
            .speed = 10,
            .x = 1,
            .y = 1,
        };
    }

    pub fn rand(self: *Maze) u32 {
        if (self.alg == .DFST) self.rnd = std.Random.DefaultPrng.init(@as(u64, @bitCast(std.time.milliTimestamp())));
        return self.rnd.random().int(u32);
    }

    pub fn setState(self: *Maze, state: MazeState) void {
        if (state == .Generating) self.reset();
        if (state == .Solving) self.clear();
        self.state = state;
    }

    pub fn setAlgorithm(self: *Maze, algorithm: MazeAlgorithm) void {
        self.alg = algorithm;
    }

    pub fn addExit(self: *Maze) void {
        var x: u32 = 0;
        var y: u32 = 0;
        if (@mod(self.rand(), 2) == 0) {
            y = @mod(self.rand(), (GRID_SIZE - 2) / 2) * 2 + 1;
            if (@mod(self.rand(), 2) == 0) {
                x = 0;
            } else {
                x = GRID_SIZE - 1;
            }
        } else {
            x = @mod(self.rand(), (GRID_SIZE - 2) / 2) * 2 + 1;
            if (@mod(self.rand(), 2) == 0) {
                y = 0;
            } else {
                y = GRID_SIZE - 1;
            }
        }
        self.grid[x][y] = .finish;
    }

    pub fn findNext(self: *Maze, x: u32, y: u32) [2]u32 {
        const dirs = [_][2]u32{ [_]u32{ 2, 0 }, [_]u32{ 2, 0 }, [_]u32{ 0, 2 }, [_]u32{ 0, 2 } };
        const gpa = std.heap.page_allocator;
        var candids = std.ArrayList([2]u32).init(gpa);

        for (0..4) |i| {
            var nx = x;
            var ny = y;
            if (@mod(i, 2) == 0) {
                if (nx <= 2) {
                    nx = 0;
                } else {
                    nx -= dirs[i][0];
                }
                if (ny <= 2) {
                    ny = 0;
                } else {
                    ny -= dirs[i][1];
                }
            } else {
                nx += dirs[i][0];
                ny += dirs[i][1];
            }

            if (nx > 0 and nx < GRID_SIZE - 1 and ny > 0 and ny < GRID_SIZE - 1 and
                self.grid[nx][ny] == .wall)
            {
                const candidate = [2]u32{ nx, ny };
                candids.append(candidate) catch unreachable;
            }
        }

        if (candids.items.len == 0) {
            candids.deinit();
            return [2]u32{ GRID_SIZE, GRID_SIZE };
        }

        const next = candids.items[@mod(self.rand(), candids.items.len)];
        candids.deinit();
        return next;
    }

    pub fn walk(self: *Maze, x: u32, y: u32) void {
        self.grid[x][y] = .path;

        while (true) {
            const next = self.findNext(x, y);
            if (next[0] == GRID_SIZE) return;

            self.grid[(x + next[0]) / 2][(y + next[1]) / 2] = .path;
            const nx = next[0];
            const ny = next[1];
            self.walk(nx, ny);
        }
    }

    pub fn stepGenerate(self: *Maze) void {
        self.walk(1, 1);
        self.addExit();
        self.setState(.Static);
    }

    pub fn setPosition(self: *Maze, pos: Pos) void {
        const x: u32 = @intFromFloat(pos.x / (CELL_SIZE + LINE_SIZE));
        const y: u32 = @intFromFloat(pos.y / (CELL_SIZE + LINE_SIZE));

        if (x < 0 or x >= GRID_SIZE or y < 0 or y >= GRID_SIZE or !self.grid[x][y].isPath() or self.state == .Solving) {
            return;
        }

        self.grid[self.x][self.y] = .path;
        self.grid[x][y] = .start;
        self.x = x;
        self.y = y;
    }

    pub fn stepSolving(self: *Maze) void {
        if (self.state != .Solving) return;
        const forward = self.direction;
        const left = self.direction.getLeft();

        const forwardPos = forward.xy();
        const forwardCell = self.grid[@intCast(@as(i32, @intCast(self.x)) + forwardPos[0])][@intCast(@as(i32, @intCast(self.y)) + forwardPos[1])];

        if (forwardCell.isPath()) {
            if (self.grid[self.x][self.y] != .start) {
                self.grid[self.x][self.y] = .solution;
            }
            self.x = @intCast(@as(i32, @intCast(self.x)) + forwardPos[0]);
            self.y = @intCast(@as(i32, @intCast(self.y)) + forwardPos[1]);

            const right = self.direction.getRight();
            const rightPos = right.xy();
            const rightCell = self.grid[@intCast(@as(i32, @intCast(self.x)) + rightPos[0])][@intCast(@as(i32, @intCast(self.y)) + rightPos[1])];

            if (rightCell.isPath()) {
                self.direction = right;
            } else if (rightCell == .finish) {
                if (self.grid[self.x][self.y] != .start) {
                    self.grid[self.x][self.y] = .solution;
                }
                self.setState(.Static);
            } else {
                const leftPos = left.xy();
                const leftCell = self.grid[@intCast(@as(i32, @intCast(self.x)) + leftPos[0])][@intCast(@as(i32, @intCast(self.y)) + leftPos[1])];
                if (leftCell == .finish) {
                    if (self.grid[self.x][self.y] != .start) {
                        self.grid[self.x][self.y] = .solution;
                    }
                    self.setState(.Static);
                }
            }
        } else if (forwardCell == .finish) {
            if (self.grid[self.x][self.y] != .start) {
                self.grid[self.x][self.y] = .solution;
            }
            self.setState(.Static);
            self.stepSolving();
        } else {
            self.direction = left;
            self.stepSolving();
        }
    }

    pub fn step(self: *Maze) void {
        switch (self.state) {
            .Generating => self.stepGenerate(),
            .Solving => for (0..self.speed) |_| {
                if (self.state != .Solving) return;
                self.stepSolving();
            },
            .Static => {},
        }
    }

    pub fn reset(self: *Maze) void {
        for (0..GRID_SIZE) |x| {
            for (0..GRID_SIZE) |y| {
                self.grid[x][y] = .wall;
            }
        }
    }

    pub fn clear(self: *Maze) void {
        for (0..GRID_SIZE) |x| {
            for (0..GRID_SIZE) |y| {
                if (self.grid[x][y] == .solution) self.grid[x][y] = .path;
            }
        }
    }

    pub fn draw(self: *Maze) void {
        const end = (CELL_SIZE + LINE_SIZE) * GRID_SIZE;

        rl.drawRectangle(-@as(i32, @intCast(LINE_SIZE / 2)), -@as(i32, @intCast(LINE_SIZE / 2)), end + LINE_SIZE, end + LINE_SIZE, LINE_COLOR);

        for (0..GRID_SIZE) |i| {
            const x = @as(i32, @intCast(i)) * (CELL_SIZE + LINE_SIZE) + LINE_SIZE / 2;
            for (0..GRID_SIZE) |j| {
                const y = @as(i32, @intCast(j)) * (CELL_SIZE + LINE_SIZE) + LINE_SIZE / 2;
                const color = switch (self.grid[i][j]) {
                    .path => PATH_COLOR,
                    .wall => WALL_COLOR,
                    .solution => SOLUTION_COLOR,
                    .finish => FINISH_COLOR,
                    .start => START_COLOR,
                };
                rl.drawRectangle(x, y, CELL_SIZE, CELL_SIZE, color);
            }
        }
    }
};

pub fn main() anyerror!void {
    var camera = rl.Camera2D{
        .target = p((CELL_SIZE + LINE_SIZE) * GRID_SIZE / 2, (CELL_SIZE + LINE_SIZE) * GRID_SIZE / 2),
        .offset = p(screenWidth / 2, screenHeight / 2),
        .rotation = 0,
        .zoom = 1,
    };

    const gridSize = (CELL_SIZE + LINE_SIZE) * GRID_SIZE;
    var maze = Maze.init();
    var mouse = rl.getMousePosition();

    rl.initWindow(screenWidth, screenHeight, "Maze Generator");
    defer rl.closeWindow();

    rl.setTargetFPS(600);

    var step_timer: f32 = 0;
    while (!rl.windowShouldClose()) {
        step_timer += rl.getFrameTime();
        if (step_timer >= 0.1) {
            step_timer = 0;
            maze.step();
        }

        rl.beginDrawing();
        defer rl.endDrawing();

        rl.clearBackground(BACKGROUND_COLOR);

        if (rl.isMouseButtonDown(.mouse_button_left)) {
            camera.target = rl.getScreenToWorld2D(camera.offset.subtract(rl.getMouseDelta()), camera);
            mouse = rl.getMousePosition();
        }

        if (rl.isMouseButtonPressed(.mouse_button_right)) {
            maze.setPosition(rl.getScreenToWorld2D(rl.getMousePosition(), camera));
        }

        const scroll = rl.getMouseWheelMove();
        if (scroll != 0) {
            const worldMouse = rl.getScreenToWorld2D(rl.getMousePosition(), camera);
            camera.offset = rl.getMousePosition();
            camera.target = worldMouse;
            camera.zoom = @max(camera.zoom + scroll * 0.05, 0.15);
        }

        // thx rozukke
        const clamp_dist = 0.55 * (1 / camera.zoom);
        const cam_xmin_limit = -(screenWidth * clamp_dist);
        const cam_xmin_clamp = cam_xmin_limit + (camera.offset.x / camera.zoom);
        const cam_xmax_limit = gridSize + screenWidth * clamp_dist;
        const cam_xmax_clamp = cam_xmax_limit - ((screenWidth - camera.offset.x) / camera.zoom);
        camera.target.x = m.clamp(camera.target.x, cam_xmin_clamp, cam_xmax_clamp);
        const cam_ymin_limit = -(screenHeight * clamp_dist);
        const cam_ymin_clamp = cam_ymin_limit + (camera.offset.y / camera.zoom);
        const cam_ymax_limit = gridSize + screenHeight * clamp_dist;
        const cam_ymax_clamp = cam_ymax_limit - ((screenHeight - camera.offset.y) / camera.zoom);
        camera.target.y = m.clamp(camera.target.y, cam_ymin_clamp, cam_ymax_clamp);

        {
            camera.begin();
            defer camera.end();

            maze.draw();
        }

        rg.guiSetIconScale(2);
        if (rg.guiButton(rl.Rectangle.init(screenWidth / 2 - 10 - 50, 20, 50, 50), rg.guiIconText(211, "")) == 1) maze.setState(.Generating);
        if (rg.guiButton(rl.Rectangle.init(screenWidth / 2 + 10, 20, 50, 50), rg.guiIconText(131, "")) == 1) maze.setState(.Solving);

        rg.guiSetIconScale(1);
        if (rg.guiButton(rl.Rectangle.init(screenWidth / 2 - 10 - 50 - 10 - 20, 20 + 15, 20, 20), rg.guiIconText(129, "")) == 1) maze.speed = @max(maze.speed, SPEED_FACTOR) - SPEED_FACTOR;
        if (rg.guiButton(rl.Rectangle.init(screenWidth / 2 + 10 + 10 + 50, 20 + 15, 20, 20), rg.guiIconText(134, "")) == 1) maze.speed += SPEED_FACTOR;

        if (rg.guiButton(rl.Rectangle.init(screenWidth / 2 - 10 - 50 + 15, 20 + 50 + 10, 20, 20), rg.guiIconText(42, "")) == 1) maze.setAlgorithm(.DFS);
        if (rg.guiButton(rl.Rectangle.init(screenWidth / 2 + 10 + 15, 20 + 50 + 10, 20, 20), rg.guiIconText(139, "")) == 1) maze.setAlgorithm(.DFST);
    }
}
