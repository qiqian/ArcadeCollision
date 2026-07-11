using System;
using System.Windows.Forms;

namespace ArcCollision.Battlefield;

internal static class Program
{
    [STAThread]
    private static void Main(string[] args)
    {
        // DpiUnaware keeps 1 logical px == 1 physical px so the window never
        // renders larger than its client size and can always fit on screen.
        Application.SetHighDpiMode(HighDpiMode.DpiUnaware);
        Application.EnableVisualStyles();
        Application.SetCompatibleTextRenderingDefault(false);
        using var form = new GameForm();
        if (args.Length == 2 && args[0] == "--screenshot")
        {
            form.SaveScreenshot(args[1]);
            return;
        }
        if (args.Length == 1 && args[0] == "--verify-renderer")
        {
            Environment.ExitCode = form.VerifyHardwarePresenter() ? 0 : 2;
            return;
        }
        Application.Run(form);
    }
}
