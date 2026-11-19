// PyMOL app launcher: collects files from Finder's "open documents" Apple Event
// and passes them to PyMOL as command-line arguments.
//
// Build: clang -fobjc-arc -O2 -framework Cocoa launcher.m -o PyMOLApp

#import <Cocoa/Cocoa.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static int g_argc;
static char **g_argv;

// Locate the Python interpreter. Preference order:
//   1. Bundled venv: <app>.app/Contents/venv/bin/python
//   2. $PYMOL_PYTHON environment variable
//   3. Compile-time fallback (-DPYMOL_PYTHON_FALLBACK=\"/path/to/python\")
// Returns NULL if no executable interpreter was found.
static const char *find_python_executable(void) {
    NSString *bundled = [NSBundle.mainBundle.bundlePath
        stringByAppendingPathComponent:@"Contents/venv/bin/python"];
    if (access(bundled.fileSystemRepresentation, X_OK) == 0)
        return strdup(bundled.fileSystemRepresentation);

    const char *env = getenv("PYMOL_PYTHON");
    if (env && *env && access(env, X_OK) == 0)
        return env;

#ifdef PYMOL_PYTHON_FALLBACK
    if (access(PYMOL_PYTHON_FALLBACK, X_OK) == 0)
        return PYMOL_PYTHON_FALLBACK;
#endif

    return NULL;
}

static void fail_no_python(void) {
    NSAlert *alert = [NSAlert new];
    alert.alertStyle = NSAlertStyleCritical;
    alert.messageText = @"Python interpreter not found";
    alert.informativeText = [NSString stringWithFormat:
        @"Expected a Python environment with PyMOL installed at:\n%@\n\n"
        @"Alternatively, set the PYMOL_PYTHON environment variable.",
        [NSBundle.mainBundle.bundlePath
            stringByAppendingPathComponent:@"Contents/venv/bin/python"]];
    [NSApp activateIgnoringOtherApps:YES];
    [alert runModal];
    exit(1);
}

static void run_and_exit(char **args) {
    execv(args[0], args);
    perror("execv");
    exit(1);
}

@interface Launcher : NSObject <NSApplicationDelegate>
@property (strong) NSMutableArray<NSString *> *files;
@end

@implementation Launcher

- (instancetype)init {
    if ((self = [super init])) _files = [NSMutableArray array];
    return self;
}

// Called before applicationDidFinishLaunching: when the app is launched
// by opening documents (double-click, drag onto icon, `open file.pdb`).
- (void)application:(NSApplication *)app openURLs:(NSArray<NSURL *> *)urls {
    for (NSURL *url in urls)
        if (url.isFileURL) [self.files addObject:url.path];
}

- (void)applicationDidFinishLaunching:(NSNotification *)note {
    const char *python = find_python_executable();
    if (!python) fail_no_python();

    char **args = calloc(3 + g_argc + self.files.count, sizeof(char *));
    int n = 0;
    args[n++] = strdup(python);
    args[n++] = strdup("-m");
    args[n++] = strdup("pymol");
    for (int i = 1; i < g_argc; ++i)
        if (strncmp(g_argv[i], "-psn_", 5) != 0)  // legacy Finder argument
            args[n++] = g_argv[i];
    for (NSString *path in self.files)
        args[n++] = strdup(path.fileSystemRepresentation);
    args[n] = NULL;
    run_and_exit(args);
}

@end

int main(int argc, char **argv) {
    g_argc = argc;
    g_argv = argv;
    @autoreleasepool {
        NSApplication *app = [NSApplication sharedApplication];
        Launcher *delegate = [Launcher new];
        app.delegate = delegate;
        [app run];
    }
    return 0;
}
