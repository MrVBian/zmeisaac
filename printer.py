import datetime
class Printer:
    COLORS = {
        "green": "\033[92m",
        "yellow": "\033[93m",
        "red": "\033[91m",
        "blue": "\033[94m",
        "cyan": "\033[96m",
        "magenta": "\033[95m",
        "white": "\033[97m"
    }
    BOLD = "\033[1m"
    RESET = "\033[0m"

    @staticmethod
    def _get_current_time():
        return datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')

    @staticmethod
    def print_normal(message):
        current_time = Printer._get_current_time()
        print(f"{Printer.COLORS['green']}{Printer.BOLD}{current_time} - {message}{Printer.RESET}")

    @staticmethod
    def print_warning(message):
        current_time = Printer._get_current_time()
        print(f"{Printer.COLORS['yellow']}{Printer.BOLD}{current_time} - {message}{Printer.RESET}")

    @staticmethod
    def print_error(message):
        current_time = Printer._get_current_time()
        print(f"{Printer.COLORS['red']}{Printer.BOLD}{current_time} - {message}{Printer.RESET}")

    @staticmethod
    def print_custom(message, color='white'):
        current_time = Printer._get_current_time()
        color_code = Printer.COLORS.get(color, Printer.COLORS['white'])
        print(f"{color_code}{Printer.BOLD}{current_time} - {message}{Printer.RESET}")

