void filter()
{
  reading = (1 - alpha) * readingM1 + alpha * reading;
}
