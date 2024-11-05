"""
Test the printer.
"""
import win32api
import win32print
from win32com.client import DispatchEx
from backend.utils import logger


def excel_to_pdf(excel_path, pdf_path):
    logger.info(f"Converting {excel_path} to {pdf_path}")

    xlApp = DispatchEx("Excel.Application")
    xlApp.Visible = False
    xlApp.DisplayAlerts = 0

    books = xlApp.Workbooks.Open(excel_path, False)
    sheet = books.Worksheets(1)

    # 设置纸张方向
    sheet.PageSetup.Orientation = 1  # 横向：2

    # 调整边距
    sheet.PageSetup.TopMargin = xlApp.CentimetersToPoints(0.2)
    sheet.PageSetup.BottomMargin = xlApp.CentimetersToPoints(0)
    sheet.PageSetup.LeftMargin = xlApp.CentimetersToPoints(0)
    sheet.PageSetup.RightMargin = xlApp.CentimetersToPoints(0)

    # 设置打印区域
    # sheet.PageSetup.PrintArea = "A1:D20"  # 根据你的数据范围调整

    # 调整页面缩放以居中并占据95%
    sheet.PageSetup.Zoom = False
    sheet.PageSetup.FitToPagesWide = 1
    sheet.PageSetup.FitToPagesTall = 1


    # 计算并设置缩放比例以占据95%
    sheet.PageSetup.Zoom = 115  # 调整缩放比例 以95%在贴纸中央

    books.ExportAsFixedFormat(0, pdf_path)
    books.Close(False)
    xlApp.Quit()


def print_pdf(pdf_path):
    logger.info(f"Printing {pdf_path}")
    win32print.SetDefaultPrinter("ZDesigner ZD421CN-300dpi ZPL")
    win32api.ShellExecute(0, "print", pdf_path, None,  ".",  0)
    logger.info(f"Printed {pdf_path}")

def print_excel(excel_path):
    # convert excel to pdf
    pdf_path = r"C:\workspace\backend-inspection\Data\fake_res.pdf"
    excel_to_pdf(excel_path, pdf_path)
    print_pdf(pdf_path)

if __name__ == "__main__":
    excel_path = r"C:\workspace\backend-inspection\Data\fake_res.xlsx"
    print_excel(excel_path)
    # pdf_path = r"C:\workspace\backend-inspection\Data\fake_res.pdf"
    # print_pdf(pdf_path)
