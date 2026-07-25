/*
 * Program.cs
 * ArcCollision Visualizer - interactive collision test and debugging tool.
 * Copyright (c) 2026 Qian Qian <qiqian82@gmail.com>. MIT License.
 */

global using ArcCollision.Wrapper;
using System;
using System.Windows.Forms;

namespace ArcCollision.Visualizer;

internal static class Program
{
    [STAThread]
    private static void Main()
    {
        Application.EnableVisualStyles();
        Application.SetCompatibleTextRenderingDefault(false);
        Application.SetHighDpiMode(HighDpiMode.SystemAware);
        Application.Run(new MainForm());
    }
}
