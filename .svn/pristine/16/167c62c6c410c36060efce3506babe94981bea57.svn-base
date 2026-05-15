
static class Utils
{
    private static Random rand = new Random();

    public static float randomFloatUnitClamped()
    {
        return (float)rand.NextDouble();
    }

    public static float randomFloatUnitUnclamped()
    {
        return (float)(rand.NextDouble() + 1.0f / 2.0f);
    }

    public static float randomFloatColor()
    {
        return (randomFloatUnitClamped() + 0.2f) / 1.2f;
    }
}